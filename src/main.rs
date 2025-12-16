/*
 * Copyright (c) 2025 noumanimpra
 * Licensed under the MIT License. See LICENSE file in the project root for full license information.
 */

use std::{
    io::{self, stdout},
    sync::mpsc::{self, Receiver, Sender},
    thread,
    time::Duration,
};

use anyhow::Result;
use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use ratatui::{
    Frame, Terminal,
    backend::CrosstermBackend,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style, Stylize},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, ListState, Paragraph},
};
use serialport::{SerialPortInfo, available_ports};

/// -----------------------------------------------------------------------------
/// Data Structures and Modes
/// -----------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode {
    PortSelection,
    BaudSelection,
    Monitoring,
}

enum PortCommand {
    Connect(String, u32),
    Disconnect,
    Exit,
}

// Messages from port thread
enum PortMessage {
    Data(String),
    Connected,
    Disconnected,
    Error(String),
}

struct LogEntry {
    timestamp: String,
    data: String,
}

struct Application {
    mode: Mode,
    available_ports: Vec<SerialPortInfo>,
    current_line_buffer: String,

    // Configuration
    baud_rate_options: Vec<u32>,
    selected_baud_rate: u32,

    // TUI state
    port_list_state: ListState,
    baud_list_state: ListState,
    log_scroll_offset: usize,

    // Data
    selected_port_name: String,
    log_buffer: Vec<LogEntry>,
    show_timestamps: bool,

    // Thread Communication
    port_command_tx: Sender<PortCommand>,
    port_message_rx: Receiver<PortMessage>,
    is_connected: bool,

    // Statistics
    total_bytes: usize,
    connection_time: Option<std::time::Instant>,
    log_area_height: usize,
}

impl Application {
    fn new() -> Result<Application> {
        let (message_tx, message_rx) = mpsc::channel();
        let (command_tx, command_rx) = mpsc::channel();

        // Start background port management thread
        thread::spawn(move || {
            port_manager_thread(command_rx, message_tx);
        });

        let ports = available_ports().unwrap_or_default();
        let baud_rates = vec![9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];

        let mut port_list_state = ListState::default();
        if !ports.is_empty() {
            port_list_state.select(Some(0));
        }

        let default_baud = 115200;
        let mut baud_list_state = ListState::default();
        if let Some(i) = baud_rates.iter().position(|&b| b == default_baud) {
            baud_list_state.select(Some(i));
        }

        Ok(Application {
            mode: Mode::PortSelection,
            available_ports: ports,
            selected_port_name: String::new(),
            baud_rate_options: baud_rates,
            selected_baud_rate: default_baud,
            port_list_state,
            baud_list_state,
            log_scroll_offset: 0,
            log_buffer: Vec::new(),
            show_timestamps: true,
            port_command_tx: command_tx,
            port_message_rx: message_rx,
            is_connected: false,
            total_bytes: 0,
            connection_time: None,
            current_line_buffer: String::new(),
            log_area_height: 0,
        })
    }

    fn scroll_log_up(&mut self) {
        if self.log_scroll_offset > 0 {
            self.log_scroll_offset -= 1;
        }
    }

    fn scroll_log_down(&mut self, max_visible: usize) {
        // Eğer buffer boşsa veya ekrana sığıyorsa işlem yapma
        if self.log_buffer.len() <= max_visible {
            return;
        }

        // Güvenli hesaplama: Toplam satır - Görünebilir alan
        let max_offset = self.log_buffer.len().saturating_sub(max_visible);

        if self.log_scroll_offset < max_offset {
            self.log_scroll_offset += 1;
        }
    }

    fn scroll_to_bottom(&mut self, max_visible: usize) {
        if self.log_buffer.len() > max_visible {
            self.log_scroll_offset = self.log_buffer.len().saturating_sub(max_visible);
        } else {
            self.log_scroll_offset = 0;
        }
    }

    fn scroll_to_top(&mut self) {
        self.log_scroll_offset = 0;
    }

    fn add_log_entry(&mut self, data: String) {
        use std::time::{SystemTime, UNIX_EPOCH};

        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();

        let total_secs = now.as_secs();
        let millis = now.subsec_millis();

        // Convert to local time (hours, minutes, seconds)
        let secs_in_day = total_secs % 86400;
        let hours = (secs_in_day / 3600) as u32;
        let minutes = ((secs_in_day % 3600) / 60) as u32;
        let seconds = (secs_in_day % 60) as u32;

        let timestamp = format!("{:02}:{:02}:{:02}.{:03}", hours, minutes, seconds, millis);

        self.log_buffer.push(LogEntry { timestamp, data });
    }

    fn toggle_timestamps(&mut self) {
        self.show_timestamps = !self.show_timestamps;
    }
}

// Standalone functions - solves borrow checker issues
fn next_selection(list_length: usize, state: &mut ListState) {
    if list_length == 0 {
        return;
    }
    let i = match state.selected() {
        Some(i) => {
            if i >= list_length - 1 {
                0
            } else {
                i + 1
            }
        }
        None => 0,
    };
    state.select(Some(i));
}

fn previous_selection(list_length: usize, state: &mut ListState) {
    if list_length == 0 {
        return;
    }
    let i = match state.selected() {
        Some(i) => {
            if i == 0 {
                list_length - 1
            } else {
                i - 1
            }
        }
        None => 0,
    };
    state.select(Some(i));
}

/// -----------------------------------------------------------------------------
/// Port Management Thread
/// -----------------------------------------------------------------------------

fn port_manager_thread(command_rx: Receiver<PortCommand>, message_tx: Sender<PortMessage>) {
    let mut port_open: Option<Box<dyn serialport::SerialPort>> = None;
    let mut serial_buf: Vec<u8> = vec![0; 4096];

    loop {
        // Check for incoming commands
        if let Ok(command) = command_rx.try_recv() {
            match command {
                PortCommand::Connect(name, baud) => {
                    match serialport::new(&name, baud)
                        .timeout(Duration::from_millis(10))
                        .open()
                    {
                        Ok(port) => {
                            port_open = Some(port);
                            let _ = message_tx.send(PortMessage::Connected);
                            let _ = message_tx.send(PortMessage::Data(format!(
                                "┌─────────────────────────────────────────┐\n│ ✓ Connected: {} @ {} baud\n└─────────────────────────────────────────┘",
                                name, baud
                            )));
                        }
                        Err(e) => {
                            let _ = message_tx
                                .send(PortMessage::Error(format!("Failed to open port: {}", e)));
                        }
                    }
                }
                PortCommand::Disconnect => {
                    if port_open.is_some() {
                        port_open = None;
                        let _ = message_tx.send(PortMessage::Disconnected);
                        let _ = message_tx.send(PortMessage::Data(format!(
                            "┌─────────────────────────────────────────┐\n│ ⊗ Disconnected\n└─────────────────────────────────────────┘"
                        )));
                    }
                }
                PortCommand::Exit => break,
            }
        }

        // Read data if port is open
        if let Some(ref mut port) = port_open {
            match port.read(serial_buf.as_mut_slice()) {
                Ok(t) if t > 0 => {
                    let received_data = String::from_utf8_lossy(&serial_buf[..t]).to_string();
                    if message_tx.send(PortMessage::Data(received_data)).is_err() {
                        break;
                    }
                }
                Err(ref e) if e.kind() != io::ErrorKind::TimedOut => {
                    let _ = message_tx.send(PortMessage::Error(format!("Read error: {}", e)));
                    port_open = None;
                    let _ = message_tx.send(PortMessage::Disconnected);
                }
                _ => {}
            }
        }
        thread::sleep(Duration::from_millis(5));
    }
}

/// -----------------------------------------------------------------------------
/// User Interface (View)
/// -----------------------------------------------------------------------------

fn draw_ui(f: &mut Frame, app: &mut Application) {
    let main_layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // Header
            Constraint::Min(0),    // Main Content
            Constraint::Length(3), // Footer
        ])
        .split(f.size());

    // --- Header ---
    let (title, title_color) = match app.mode {
        Mode::PortSelection => ("🔌 SERIAL PORT MONITOR - PORT SELECTION", Color::Cyan),
        Mode::BaudSelection => ("⚡ SERIAL PORT MONITOR - BAUD RATE", Color::Yellow),
        Mode::Monitoring => {
            return draw_monitoring_layout(f, app, main_layout);
        }
    };

    let header = Paragraph::new(title)
        .alignment(Alignment::Center)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(Style::default().fg(title_color))
                .border_type(ratatui::widgets::BorderType::Rounded),
        )
        .style(
            Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD),
        );

    f.render_widget(header, main_layout[0]);

    // --- Main Content ---
    match app.mode {
        Mode::PortSelection => {
            draw_port_selection(f, main_layout[1], app);
        }
        Mode::BaudSelection => {
            draw_baud_selection(f, main_layout[1], app);
        }
        Mode::Monitoring => {}
    }

    // --- Footer ---
    draw_footer(f, main_layout[2], app);
}

fn draw_monitoring_layout(f: &mut Frame, app: &mut Application, areas: std::rc::Rc<[Rect]>) {
    // Custom header for monitoring mode
    let status_text = if app.is_connected {
        "● CONNECTED"
    } else {
        "○ DISCONNECTED"
    };
    let status_color = if app.is_connected {
        Color::Green
    } else {
        Color::Red
    };

    let timestamp_indicator = if app.show_timestamps {
        "🕐 ON"
    } else {
        "🕐 OFF"
    };
    let timestamp_color = if app.show_timestamps {
        Color::Green
    } else {
        Color::DarkGray
    };

    let header = Paragraph::new(Line::from(vec![
        Span::styled(
            "📡 SERIAL PORT MONITOR",
            Style::default().fg(Color::Cyan).bold(),
        ),
        Span::raw("  "),
        Span::styled(status_text, Style::default().fg(status_color).bold()),
        Span::raw("  │  "),
        Span::styled("Timestamps: ", Style::default().fg(Color::Gray)),
        Span::styled(
            timestamp_indicator,
            Style::default().fg(timestamp_color).bold(),
        ),
    ]))
    .alignment(Alignment::Center)
    .block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Cyan))
            .border_type(ratatui::widgets::BorderType::Rounded),
    );

    f.render_widget(header, areas[0]);

    draw_monitoring_mode(f, areas[1], app);
    draw_footer(f, areas[2], app);
}

fn draw_port_selection(f: &mut Frame, area: Rect, app: &mut Application) {
    if app.available_ports.is_empty() {
        let message = Paragraph::new(vec![
            Line::from(""),
            Line::from(Span::styled(
                "⚠ No Serial Ports Found",
                Style::default().fg(Color::Red).bold(),
            )),
            Line::from(""),
            Line::from("Please connect a device and restart the application."),
        ])
        .alignment(Alignment::Center)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(ratatui::widgets::BorderType::Rounded)
                .title(" Warning ")
                .title_style(Style::default().fg(Color::Red)),
        );
        f.render_widget(message, area);
        return;
    }

    let selected_index = app.port_list_state.selected();

    let port_items: Vec<ListItem> = app
        .available_ports
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let port_type = match p.port_type {
                serialport::SerialPortType::UsbPort(_) => "USB",
                serialport::SerialPortType::BluetoothPort => "Bluetooth",
                serialport::SerialPortType::PciPort => "PCI",
                _ => "Other",
            };

            let content = if Some(i) == selected_index {
                format!("▶ {} ({})", p.port_name, port_type)
            } else {
                format!("  {} ({})", p.port_name, port_type)
            };

            let style = if Some(i) == selected_index {
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::Cyan)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::White)
            };

            ListItem::new(content).style(style)
        })
        .collect();

    let port_list = List::new(port_items).block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Green))
            .border_type(ratatui::widgets::BorderType::Rounded)
            .title(" Available Ports ")
            .title_style(Style::default().fg(Color::Green).bold()),
    );

    f.render_stateful_widget(port_list, area, &mut app.port_list_state);
}

fn draw_baud_selection(f: &mut Frame, area: Rect, app: &mut Application) {
    let selected_index = app.baud_list_state.selected();

    let baud_items: Vec<ListItem> = app
        .baud_rate_options
        .iter()
        .enumerate()
        .map(|(i, b)| {
            let recommendation = match b {
                9600 => " (Standard)",
                115200 => " (Recommended)",
                _ => "",
            };

            let content = if Some(i) == selected_index {
                format!("▶ {} baud{}", b, recommendation)
            } else {
                format!("  {} baud{}", b, recommendation)
            };

            let style = if Some(i) == selected_index {
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::Yellow)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::White)
            };

            ListItem::new(content).style(style)
        })
        .collect();

    let baud_list = List::new(baud_items).block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Yellow))
            .border_type(ratatui::widgets::BorderType::Rounded)
            .title(format!(
                " Port: {} - Select Baud Rate ",
                app.selected_port_name
            ))
            .title_style(Style::default().fg(Color::Yellow).bold()),
    );

    f.render_stateful_widget(baud_list, area, &mut app.baud_list_state);
}

fn draw_monitoring_mode(f: &mut Frame, area: Rect, app: &mut Application) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(6), // İstatistik alanı
            Constraint::Min(0),    // Log alanı
        ])
        .split(area);

    // --- İstatistik Alanı ---
    let connection_duration = if let Some(time) = app.connection_time {
        let elapsed = time.elapsed().as_secs();
        format!(
            "{}:{:02}:{:02}",
            elapsed / 3600,
            (elapsed % 3600) / 60,
            elapsed % 60
        )
    } else {
        "00:00:00".to_string()
    };

    let stats = vec![
        Line::from(vec![
            Span::styled("  Port: ", Style::default().fg(Color::Gray)),
            Span::styled(
                &app.selected_port_name,
                Style::default().fg(Color::Cyan).bold(),
            ),
            Span::raw("  │  "),
            Span::styled("Baud Rate: ", Style::default().fg(Color::Gray)),
            Span::styled(
                format!("{}", app.selected_baud_rate),
                Style::default().fg(Color::Yellow).bold(),
            ),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("  Uptime: ", Style::default().fg(Color::Gray)),
            Span::styled(
                &connection_duration,
                Style::default().fg(Color::Magenta).bold(),
            ),
            Span::raw("  │  "),
            Span::styled("Data Received: ", Style::default().fg(Color::Gray)),
            Span::styled(
                format!("{} bytes", app.total_bytes),
                Style::default().fg(Color::Green).bold(),
            ),
            Span::raw("  │  "),
            Span::styled("Lines: ", Style::default().fg(Color::Gray)),
            Span::styled(
                format!("{}", app.log_buffer.len()),
                Style::default().fg(Color::Blue).bold(),
            ),
        ]),
    ];

    let stats_widget = Paragraph::new(stats).block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Magenta))
            .border_type(ratatui::widgets::BorderType::Rounded)
            .title(" 📊 Statistics ")
            .title_style(Style::default().fg(Color::Magenta).bold()),
    );

    f.render_widget(stats_widget, chunks[0]);

    // --- Log Alanı ---

    // 1. Gerçek görünür yüksekliği hesapla (Kenarlıklar için -2)
    let max_visible = (chunks[1].height as usize).saturating_sub(2);

    // 2. KRİTİK DÜZELTME: Bu yüksekliği struct içine kaydet.
    // Böylece main döngüsü "PageDown" veya "Down" tuşuna basıldığında
    // kaç satır kaydıracağını bilecek.
    app.log_area_height = max_visible;

    let log_items: Vec<ListItem> = app
        .log_buffer
        .iter()
        .skip(app.log_scroll_offset)
        .take(max_visible)
        .map(|entry| {
            let display_text = if app.show_timestamps {
                format!("[{}] {}", entry.timestamp, entry.data)
            } else {
                entry.data.clone()
            };
            ListItem::new(display_text).style(Style::default().fg(Color::White))
        })
        .collect();

    let scroll_info = if app.log_buffer.len() > max_visible {
        let current_line = app.log_scroll_offset + max_visible;
        let total_lines = app.log_buffer.len();
        format!(" [{}/{}] ", current_line.min(total_lines), total_lines)
    } else if app.log_buffer.is_empty() {
        " [Waiting for data...] ".to_string()
    } else {
        format!(" [All {} lines] ", app.log_buffer.len())
    };

    let log_list = List::new(log_items).block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(if app.is_connected {
                Color::Green
            } else {
                Color::Red
            }))
            .border_type(ratatui::widgets::BorderType::Rounded)
            .title(format!(" 📜 Data Stream {}", scroll_info))
            .title_style(Style::default().fg(Color::Cyan).bold()),
    );

    f.render_widget(log_list, chunks[1]);
}

fn draw_footer(f: &mut Frame, area: Rect, app: &Application) {
    let keys = match app.mode {
        Mode::PortSelection => "↑↓: Navigate  │  ENTER: Select  │  Q/ESC: Quit",
        Mode::BaudSelection => "↑↓: Navigate  │  ENTER: Select  │  Q/ESC: Quit",
        Mode::Monitoring => {
            "S: Connect/Disconnect  │  T: Toggle Timestamps  │  C: Clear  │  ↑↓: Scroll  │  Q/ESC: Quit"
        }
    };

    let footer = Paragraph::new(Line::from(vec![
        Span::raw("  "),
        Span::styled(keys, Style::default().fg(Color::Cyan)),
    ]))
    .block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::DarkGray))
            .border_type(ratatui::widgets::BorderType::Rounded),
    )
    .alignment(Alignment::Center);

    f.render_widget(footer, area);
}

/// -----------------------------------------------------------------------------
/// Main (Controller)
/// -----------------------------------------------------------------------------

fn main() -> Result<()> {
    enable_raw_mode()?;
    execute!(stdout(), EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout());
    let mut terminal = Terminal::new(backend)?;

    let mut app = Application::new()?;
    let mut should_quit = false;

    while !should_quit {
        terminal.draw(|f| draw_ui(f, &mut app))?;

        // Process messages from port thread
        let mut new_data_arrived = false;
        // Main fonksiyonu içindeki while döngüsünde:
        while let Ok(message) = app.port_message_rx.try_recv() {
            match message {
                PortMessage::Data(data) => {
                    app.total_bytes += data.len();

                    // 1. Gelen veriyi geçici buffer'a ekle
                    app.current_line_buffer.push_str(&data);

                    // 2. Buffer içinde '\n' (yeni satır) var mı diye bak
                    while let Some(pos) = app.current_line_buffer.find('\n') {
                        // '\n' karakterine kadar olan kısmı kesip al
                        // drain kullanarak buffer'ın başını alıyoruz, kalanı buffer'da kalıyor
                        let line: String = app.current_line_buffer.drain(..=pos).collect();

                        // Satır sonundaki boşlukları (\n, \r) temizle
                        let trimmed_line = line.trim_end().to_string();

                        // Eğer satır boş değilse loglara ekle
                        if !trimmed_line.is_empty() {
                            app.add_log_entry(trimmed_line);
                            new_data_arrived = true;
                        }
                    }
                    // Döngü bittiğinde buffer'da '\n' kalmamıştır.
                    // Yarım kalan veri (örn: "Conv") buffer'da bekler,
                    // bir sonraki veri geldiğinde ("erted") onunla birleşir.
                }
                PortMessage::Connected => {
                    app.is_connected = true;
                    app.connection_time = Some(std::time::Instant::now());
                    // Bağlandığında buffer'ı temizlemek iyi bir fikirdir
                    app.current_line_buffer.clear();
                }
                PortMessage::Disconnected => {
                    app.is_connected = false;
                    app.connection_time = None;
                }
                PortMessage::Error(error) => {
                    app.add_log_entry(format!("✗ ERROR: {}", error));
                    new_data_arrived = true;
                }
            }
        }

        // Auto-scroll to bottom when new data arrives and we're in monitoring mode
        if new_data_arrived && app.mode == Mode::Monitoring && app.is_connected {
            // Force scroll to the very bottom
            if app.log_buffer.len() > 0 {
                app.log_scroll_offset = app.log_buffer.len().saturating_sub(app.log_area_height);
            }
        }

        // Handle input
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => {
                            app.port_command_tx.send(PortCommand::Exit).ok();
                            should_quit = true;
                        }
                        _ => match app.mode {
                            Mode::PortSelection => match key.code {
                                KeyCode::Up | KeyCode::Char('k') => {
                                    let len = app.available_ports.len();
                                    previous_selection(len, &mut app.port_list_state);
                                }
                                KeyCode::Down | KeyCode::Char('j') => {
                                    let len = app.available_ports.len();
                                    next_selection(len, &mut app.port_list_state);
                                }
                                KeyCode::Enter => {
                                    if let Some(i) = app.port_list_state.selected() {
                                        app.selected_port_name =
                                            app.available_ports[i].port_name.clone();
                                        app.mode = Mode::BaudSelection;
                                    }
                                }
                                _ => {}
                            },
                            Mode::BaudSelection => match key.code {
                                KeyCode::Up | KeyCode::Char('k') => {
                                    let len = app.baud_rate_options.len();
                                    previous_selection(len, &mut app.baud_list_state);
                                }
                                KeyCode::Down | KeyCode::Char('j') => {
                                    let len = app.baud_rate_options.len();
                                    next_selection(len, &mut app.baud_list_state);
                                }
                                KeyCode::Enter => {
                                    if let Some(i) = app.baud_list_state.selected() {
                                        app.selected_baud_rate = app.baud_rate_options[i];
                                        app.mode = Mode::Monitoring;
                                    }
                                }
                                _ => {}
                            },
                            Mode::Monitoring => match key.code {
                                KeyCode::Char('s') | KeyCode::Char('S') => {
                                    if app.is_connected {
                                        app.port_command_tx.send(PortCommand::Disconnect).ok();
                                    } else {
                                        app.port_command_tx
                                            .send(PortCommand::Connect(
                                                app.selected_port_name.clone(),
                                                app.selected_baud_rate,
                                            ))
                                            .ok();
                                    }
                                }
                                KeyCode::Char('t') | KeyCode::Char('T') => {
                                    app.toggle_timestamps();
                                }
                                KeyCode::Char('c') | KeyCode::Char('C') => {
                                    app.log_buffer.clear();
                                    app.log_scroll_offset = 0;
                                    app.total_bytes = 0;
                                }
                                KeyCode::Up => app.scroll_log_up(),
                                KeyCode::Down => app.scroll_log_down(app.log_area_height),
                                KeyCode::Home => app.scroll_to_top(),
                                KeyCode::End => app.scroll_to_bottom(app.log_area_height),
                                KeyCode::PageUp => {
                                    for _ in 0..10 {
                                        app.scroll_log_up();
                                    }
                                }
                                KeyCode::PageDown => {
                                    for _ in 0..10 {
                                        app.scroll_log_down(app.log_area_height); // Burayı da güncelleyin
                                    }
                                }
                                _ => {}
                            },
                        },
                    }
                }
            }
        }
    }

    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    disable_raw_mode()?;

    Ok(())
}
