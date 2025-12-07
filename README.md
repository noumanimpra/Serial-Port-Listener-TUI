# 🔌 Serial Port Listener TUI

A lightweight, cross-platform, Terminal User Interface (TUI) for monitoring serial ports, built with Rust and Ratatui.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Rust](https://img.shields.io/badge/built_with-Rust-orange.svg)
![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20macOS%20%7C%20Windows-lightgrey)

## ✨ Features

- **Port Discovery:** Automatically lists available serial (USB/Bluetooth/PCI) ports.
- **Baud Rate Selection:** Supports standard baud rates from 9600 to 921600.
- **Real-time Monitoring:** Asynchronous data reading without freezing the UI.
- **Timestamps:** Toggleable high-precision timestamps for incoming data.
- **Statistics:** Tracks connection uptime, total bytes received, and line counts.
- **Log Management:** Auto-scrolling, manual navigation, and clear buffer functions.
- **Cross-Platform:** Works seamlessly on Linux, macOS, and Windows.

## 🚀 Installation

### Option 1: Install via Cargo (Recommended)

If you have Rust installed, you can build and install directly from the source:

```bash
git clone https://github.com/noumanimpra/Serial-Port-Listener-TUI.git
cd Serial-Port-Listener-TUI
cargo install --path  .
```

### Option 2: Build from Source

```bash
# Clone the repository

git clone https://github.com/kullaniciadi/Serial-Port-Listener-TUI.git
cd Serial-Port-Listener-TUI

# Build for release

cargo build --release

# Run the binary

./target/release/Serial-Port-Listener-TUI
```

## 🖥️ Operating System Specific Setup

### 🐧 Linux

On Linux, your user must have permission to access serial ports (usually `/dev/ttyUSB0` or `/dev/ttyACM0`).

1. Add your user to the `dialout` (or `uucp` on Arch Linux) group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
2. **Log out and log back in** for changes to take effect.

### 🍎 macOS

macOS typically handles serial drivers automatically. However, if you don't see your device:

1. Ensure the necessary drivers (like CH340 or CP210x) are installed for your specific hardware.
2. You may need to allow the accessory to connect in System Settings.

### 🪟 Windows

No special setup is required. Windows usually assigns a `COM` port automatically.

## 🎮 Controls

| Context        | Key                   | Action                       |
| -------------- | --------------------- | ---------------------------- |
| **Navigation** | `↑` / `↓` / `k` / `j` | Move selection / Scroll logs |
|                | `Home` / \`End\`      | Scroll to top / bottom       |
|                | `PageUp` / `PageDown` | Fast scroll                  |
| **Selection**  | `Enter`               | Confirm Port / Baud Rate     |
| **Monitoring** | `s`                   | Start / Stop Connection      |
|                | `t`                   | Toggle Timestamps            |
|                | `c`                   | Clear Log Buffer             |
| **General**    | `q` or `Esc           | Quit Application             |

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
