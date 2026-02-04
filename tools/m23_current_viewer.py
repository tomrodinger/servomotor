#!/usr/bin/env python3
"""
M23 Current Streaming Viewer
============================

Real-time visualization of M23 motor current telemetry data using PyQtGraph.

Features:
- Dual graphs: Current (Phase A/B) and PWM (Phase A/B)
- Toggle streaming with 's' button
- Freeze/zoom mode for detailed waveform inspection
- Statistics display with min/max/RMS metrics
- 60 Hz smooth visualization with GPU acceleration

Usage:
  python m23_current_viewer.py [OPTIONS]

Options:
  --port PORT     Serial port (default: auto-detect or interactive)
  --baud RATE     Baud rate (default: 5000000)
  --window SEC    Time window to display in seconds (default: 1.0)
  -P              Interactive port selection
  --help          Show this help message

Requirements:
  pip install pyserial numpy pyqtgraph PyQt6
  (or PyQt5)
"""

import argparse
import sys
import time
import threading
import signal
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("Error: numpy not installed. Run: pip install numpy")
    sys.exit(1)

# Try PyQt6 first, fall back to PyQt5
QT_VERSION = None
try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QPushButton, QComboBox, QGroupBox, QGridLayout,
        QSplitter, QFrame, QSizePolicy
    )
    from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
    from PyQt6.QtGui import QFont
    QT_VERSION = 6
except ImportError:
    try:
        from PyQt5.QtWidgets import (
            QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
            QLabel, QPushButton, QComboBox, QGroupBox, QGridLayout,
            QSplitter, QFrame, QSizePolicy
        )
        from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
        from PyQt5.QtGui import QFont
        QT_VERSION = 5
    except ImportError:
        print("Error: Neither PyQt6 nor PyQt5 installed.")
        print("  Try: pip install PyQt5 pyqtgraph")
        sys.exit(1)

try:
    import pyqtgraph as pg
except ImportError:
    print("Error: pyqtgraph not installed. Run: pip install pyqtgraph")
    sys.exit(1)

# Qt5/Qt6 Compatibility
if QT_VERSION == 6:
    AlignRight = Qt.AlignmentFlag.AlignRight
    Horizontal = Qt.Orientation.Horizontal
else:
    AlignRight = Qt.AlignRight
    Horizontal = Qt.Horizontal

# Import protocol definitions
from m23_telemetry_protocol import (
    FrameParser, M23TelemetryFrame,
    TELEMETRY_FRAME_SIZE, DEFAULT_BAUD, DEBUG_BAUD, SAMPLE_RATE_HZ,
    ADC_MAX_VALUE, PWM_PERIOD_TIM1,
    COLOR_CURRENT_A, COLOR_CURRENT_B, COLOR_PWM_A, COLOR_PWM_B
)

# PyQtGraph configuration
pg.setConfigOptions(antialias=True, useOpenGL=False)


# ==============================================================================
# Ring Buffer for Telemetry Data
# ==============================================================================

class TelemetryRingBuffer:
    """Thread-safe circular buffer for telemetry data."""

    def __init__(self, capacity: int = 100000):
        self.capacity = capacity
        self.current_a = np.zeros(capacity, dtype=np.int16)
        self.current_b = np.zeros(capacity, dtype=np.int16)
        self.pwm_a = np.zeros(capacity, dtype=np.int16)
        self.pwm_b = np.zeros(capacity, dtype=np.int16)
        self.write_idx = 0
        self.sample_count = 0
        self.lock = threading.Lock()
        self.paused = False

    def add_frames(self, frames: list) -> bool:
        """Add multiple frames to the buffer."""
        if self.paused or not frames:
            return True

        n = len(frames)
        current_a_arr = np.array([f.current_a for f in frames], dtype=np.int16)
        current_b_arr = np.array([f.current_b for f in frames], dtype=np.int16)
        pwm_a_arr = np.array([f.pwm_a for f in frames], dtype=np.int16)
        pwm_b_arr = np.array([f.pwm_b for f in frames], dtype=np.int16)

        with self.lock:
            start = self.write_idx
            end = self.write_idx + n
            cap = self.capacity
            start_mod = start % cap
            end_mod = end % cap

            if start_mod < end_mod or end_mod == 0:
                sl = slice(start_mod, start_mod + n if end_mod == 0 else end_mod)
                self.current_a[sl] = current_a_arr
                self.current_b[sl] = current_b_arr
                self.pwm_a[sl] = pwm_a_arr
                self.pwm_b[sl] = pwm_b_arr
            else:
                # Wraps around
                n1 = cap - start_mod
                self.current_a[start_mod:cap] = current_a_arr[:n1]
                self.current_b[start_mod:cap] = current_b_arr[:n1]
                self.pwm_a[start_mod:cap] = pwm_a_arr[:n1]
                self.pwm_b[start_mod:cap] = pwm_b_arr[:n1]

                self.current_a[:end_mod] = current_a_arr[n1:]
                self.current_b[:end_mod] = current_b_arr[n1:]
                self.pwm_a[:end_mod] = pwm_a_arr[n1:]
                self.pwm_b[:end_mod] = pwm_b_arr[n1:]

            self.write_idx = end
            self.sample_count += n

        return True

    def get_window(self, window_sec: float) -> tuple:
        """Get most recent data within the specified time window."""
        with self.lock:
            if self.write_idx == 0:
                return None, None, None, None, None

            n_available = min(self.write_idx, self.capacity)
            n_req = max(1, int(window_sec * SAMPLE_RATE_HZ))
            n = min(n_available, n_req)

            end = self.write_idx
            start = end - n
            idxs = (np.arange(start, end) % self.capacity).astype(np.int64)

            # Time axis: negative values (past) to 0 (now)
            t = np.arange(-n + 1, 1, dtype=np.float64) / float(SAMPLE_RATE_HZ)

            return (
                t,
                self.current_a[idxs].copy(),
                self.current_b[idxs].copy(),
                self.pwm_a[idxs].copy(),
                self.pwm_b[idxs].copy()
            )

    def get_stats(self, window_sec: float = 1.0) -> dict:
        """Get statistics for the data in buffer."""
        t, current_a, current_b, pwm_a, pwm_b = self.get_window(window_sec)

        if t is None or len(t) == 0:
            return None

        return {
            'current_a_min': int(np.min(current_a)),
            'current_a_max': int(np.max(current_a)),
            'current_a_rms': float(np.sqrt(np.mean(current_a.astype(np.float32)**2))),
            'current_b_min': int(np.min(current_b)),
            'current_b_max': int(np.max(current_b)),
            'current_b_rms': float(np.sqrt(np.mean(current_b.astype(np.float32)**2))),
            'pwm_a_min': int(np.min(pwm_a)),
            'pwm_a_max': int(np.max(pwm_a)),
            'pwm_b_min': int(np.min(pwm_b)),
            'pwm_b_max': int(np.max(pwm_b)),
            'n_samples': len(t),
        }


# ==============================================================================
# Serial Reader Thread
# ==============================================================================

class SerialReader(QObject):
    """Background thread for reading serial data."""

    frames_received = pyqtSignal(int)
    connection_status = pyqtSignal(str)
    error_signal = pyqtSignal(str)

    def __init__(self, buffer: TelemetryRingBuffer, stream_baud: int = DEFAULT_BAUD):
        super().__init__()
        self.buffer = buffer
        self.parser = FrameParser()
        self.serial_port = None
        self.running = False
        self.thread = None
        self.stream_baud = stream_baud

    def connect(self, port: str, baud: int):
        """Connect to serial port."""
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.serial_port.reset_input_buffer()
            self.connection_status.emit(f"Connected to {port}")
            return True
        except serial.SerialException as e:
            self.error_signal.emit(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.connection_status.emit("Disconnected")

    def start_reading(self):
        """Start the reader thread."""
        if self.serial_port and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()

    def stop_reading(self):
        """Stop the reader thread."""
        self.running = False

    def send_toggle_streaming(self):
        """Send 's' to toggle streaming."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b's')
            except serial.SerialException as e:
                self.error_signal.emit(f"Write error: {e}")

    def send_start_streaming_sequence(self):
        """Send init sequence: '5' at DEBUG_BAUD, then 's' at stream baud."""
        if self.serial_port and self.serial_port.is_open:
            try:
                # Step 1: switch to debug baud and send '5'
                self.serial_port.baudrate = DEBUG_BAUD
                self.serial_port.reset_input_buffer()
                time.sleep(0.1)
                self.serial_port.write(b'5')
                time.sleep(0.1)

                # Step 2: switch to streaming baud and send 's'
                self.serial_port.baudrate = self.stream_baud
                time.sleep(0.05)
                self.serial_port.reset_input_buffer()
                time.sleep(0.05)
                self.serial_port.write(b's')
                time.sleep(0.05)
            except serial.SerialException as e:
                self.error_signal.emit(f"Write error: {e}")

    def _read_loop(self):
        """Background read loop."""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    frames = self.parser.feed(data)
                    if frames:
                        self.buffer.add_frames(frames)
                        self.frames_received.emit(len(frames))
                else:
                    time.sleep(0.001)
            except serial.SerialException:
                break
            except Exception:
                break


# ==============================================================================
# Main Window
# ==============================================================================

class M23CurrentViewer(QMainWindow):
    """Main application window."""

    def __init__(self, port: str = None, baud: int = DEFAULT_BAUD,
                 window_sec: float = 1.0):
        super().__init__()
        self.window_sec = window_sec
        self.baud = baud
        self.initial_port = port

        self.buffer = TelemetryRingBuffer()
        self.reader = SerialReader(self.buffer, stream_baud=self.baud)
        self.streaming = False
        self.frozen = False
        self.total_frames = 0

        self.init_ui()
        self.init_plots()
        self.connect_signals()

        # Update timer (60 Hz)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(16)  # ~60 fps

        # Stats timer (2 Hz)
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.update_stats)
        self.stats_timer.start(500)

        # Auto-connect if port specified
        if self.initial_port:
            self.port_combo.setCurrentText(self.initial_port)
            self.connect_clicked()

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle("M23 Current Streaming Viewer")
        self.setMinimumSize(1000, 700)

        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # Control bar
        control_bar = QHBoxLayout()

        # Port selection
        port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        self.refresh_ports()

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.refresh_ports)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_clicked)

        # Streaming control
        self.stream_btn = QPushButton("Toggle Streaming (s)")
        self.stream_btn.clicked.connect(self.toggle_streaming)
        self.stream_btn.setEnabled(False)

        # Freeze button
        self.freeze_btn = QPushButton("Freeze")
        self.freeze_btn.setCheckable(True)
        self.freeze_btn.clicked.connect(self.toggle_freeze)

        # Status label
        self.status_label = QLabel("Disconnected")
        self.status_label.setMinimumWidth(200)

        control_bar.addWidget(port_label)
        control_bar.addWidget(self.port_combo)
        control_bar.addWidget(refresh_btn)
        control_bar.addWidget(self.connect_btn)
        control_bar.addSpacing(20)
        control_bar.addWidget(self.stream_btn)
        control_bar.addWidget(self.freeze_btn)
        control_bar.addStretch()
        control_bar.addWidget(self.status_label)

        main_layout.addLayout(control_bar)

        # Splitter for graphs and stats
        splitter = QSplitter(Horizontal)

        # Graph container
        graph_widget = QWidget()
        graph_layout = QVBoxLayout(graph_widget)
        graph_layout.setContentsMargins(0, 0, 0, 0)

        # Current graph
        self.current_plot = pg.PlotWidget(title="Motor Current")
        self.current_plot.setLabel('left', 'ADC Counts')
        self.current_plot.setLabel('bottom', 'Time', units='s')
        self.current_plot.showGrid(x=True, y=True, alpha=0.3)
        self.current_plot.setYRange(0, ADC_MAX_VALUE)
        self.current_plot.addLegend()

        # PWM graph
        self.pwm_plot = pg.PlotWidget(title="PWM Duty")
        self.pwm_plot.setLabel('left', 'PWM Value')
        self.pwm_plot.setLabel('bottom', 'Time', units='s')
        self.pwm_plot.showGrid(x=True, y=True, alpha=0.3)
        self.pwm_plot.setYRange(0, PWM_PERIOD_TIM1)
        self.pwm_plot.addLegend()

        graph_layout.addWidget(self.current_plot)
        graph_layout.addWidget(self.pwm_plot)

        # Statistics panel
        stats_widget = QGroupBox("Statistics")
        stats_layout = QGridLayout(stats_widget)
        stats_widget.setMaximumWidth(250)

        self.stats_labels = {}
        row = 0

        # Current A stats
        stats_layout.addWidget(QLabel("Current A:"), row, 0, 1, 2)
        row += 1
        for stat in ['min', 'max', 'rms']:
            label = QLabel(f"  {stat.upper()}:")
            value = QLabel("--")
            value.setAlignment(AlignRight)
            stats_layout.addWidget(label, row, 0)
            stats_layout.addWidget(value, row, 1)
            self.stats_labels[f'current_a_{stat}'] = value
            row += 1

        # Current B stats
        stats_layout.addWidget(QLabel("Current B:"), row, 0, 1, 2)
        row += 1
        for stat in ['min', 'max', 'rms']:
            label = QLabel(f"  {stat.upper()}:")
            value = QLabel("--")
            value.setAlignment(AlignRight)
            stats_layout.addWidget(label, row, 0)
            stats_layout.addWidget(value, row, 1)
            self.stats_labels[f'current_b_{stat}'] = value
            row += 1

        # PWM A stats
        stats_layout.addWidget(QLabel("PWM A:"), row, 0, 1, 2)
        row += 1
        for stat in ['min', 'max']:
            label = QLabel(f"  {stat.upper()}:")
            value = QLabel("--")
            value.setAlignment(AlignRight)
            stats_layout.addWidget(label, row, 0)
            stats_layout.addWidget(value, row, 1)
            self.stats_labels[f'pwm_a_{stat}'] = value
            row += 1

        # PWM B stats
        stats_layout.addWidget(QLabel("PWM B:"), row, 0, 1, 2)
        row += 1
        for stat in ['min', 'max']:
            label = QLabel(f"  {stat.upper()}:")
            value = QLabel("--")
            value.setAlignment(AlignRight)
            stats_layout.addWidget(label, row, 0)
            stats_layout.addWidget(value, row, 1)
            self.stats_labels[f'pwm_b_{stat}'] = value
            row += 1

        # Frame count
        stats_layout.addWidget(QLabel(""), row, 0)  # Spacer
        row += 1
        stats_layout.addWidget(QLabel("Frames:"), row, 0)
        self.frame_count_label = QLabel("0")
        self.frame_count_label.setAlignment(AlignRight)
        stats_layout.addWidget(self.frame_count_label, row, 1)
        row += 1

        stats_layout.addWidget(QLabel("Rate:"), row, 0)
        self.frame_rate_label = QLabel("0 Hz")
        self.frame_rate_label.setAlignment(AlignRight)
        stats_layout.addWidget(self.frame_rate_label, row, 1)
        row += 1

        stats_layout.addWidget(QLabel("Sync Err:"), row, 0)
        self.sync_err_label = QLabel("0")
        self.sync_err_label.setAlignment(AlignRight)
        stats_layout.addWidget(self.sync_err_label, row, 1)

        stats_layout.setRowStretch(row + 1, 1)

        splitter.addWidget(graph_widget)
        splitter.addWidget(stats_widget)
        splitter.setSizes([800, 200])

        main_layout.addWidget(splitter)

        # Apply dark theme
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QPushButton {
                background-color: #404040;
                border: 1px solid #555555;
                padding: 5px 15px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #505050;
            }
            QPushButton:pressed {
                background-color: #606060;
            }
            QPushButton:checked {
                background-color: #2ecc71;
                color: #000000;
            }
            QPushButton:disabled {
                background-color: #353535;
                color: #666666;
            }
            QComboBox {
                background-color: #404040;
                border: 1px solid #555555;
                padding: 5px;
                border-radius: 3px;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #555555;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLabel {
                color: #cccccc;
            }
        """)

    def init_plots(self):
        """Initialize plot curves."""
        # Current plot curves
        self.current_a_curve = self.current_plot.plot(
            pen=pg.mkPen(COLOR_CURRENT_A, width=1),
            name='Phase A'
        )
        self.current_b_curve = self.current_plot.plot(
            pen=pg.mkPen(COLOR_CURRENT_B, width=1),
            name='Phase B'
        )

        # PWM plot curves
        self.pwm_a_curve = self.pwm_plot.plot(
            pen=pg.mkPen(COLOR_PWM_A, width=1),
            name='Phase A'
        )
        self.pwm_b_curve = self.pwm_plot.plot(
            pen=pg.mkPen(COLOR_PWM_B, width=1),
            name='Phase B'
        )

    def connect_signals(self):
        """Connect signals."""
        self.reader.frames_received.connect(self.on_frames_received)
        self.reader.connection_status.connect(self.on_connection_status)
        self.reader.error_signal.connect(self.on_error)

    def refresh_ports(self):
        """Refresh available serial ports."""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def connect_clicked(self):
        """Handle connect/disconnect button."""
        if self.reader.serial_port and self.reader.serial_port.is_open:
            self.reader.disconnect()
            self.connect_btn.setText("Connect")
            self.stream_btn.setEnabled(False)
            self.streaming = False
        else:
            port = self.port_combo.currentText()
            if port and self.reader.connect(port, self.baud):
                self.reader.start_reading()
                self.connect_btn.setText("Disconnect")
                self.stream_btn.setEnabled(True)

    def toggle_streaming(self):
        """Toggle streaming on/off."""
        if not self.streaming:
            self.reader.send_start_streaming_sequence()
            self.streaming = True
            self.stream_btn.setText("Stop Streaming (s)")
            self.reader.parser.reset()  # Reset parser for clean start
        else:
            self.reader.send_toggle_streaming()
            self.streaming = False
            self.stream_btn.setText("Toggle Streaming (s)")

    def toggle_freeze(self):
        """Toggle freeze mode."""
        self.frozen = self.freeze_btn.isChecked()
        self.buffer.paused = self.frozen
        if self.frozen:
            self.freeze_btn.setText("Unfreeze")
        else:
            self.freeze_btn.setText("Freeze")

    def on_frames_received(self, count: int):
        """Handle frames received signal."""
        self.total_frames += count

    def on_connection_status(self, status: str):
        """Handle connection status change."""
        self.status_label.setText(status)

    def on_error(self, error: str):
        """Handle error signal."""
        self.status_label.setText(f"Error: {error}")

    def update_plots(self):
        """Update plot data."""
        if self.frozen:
            return

        data = self.buffer.get_window(self.window_sec)
        if data[0] is None:
            return

        t, current_a, current_b, pwm_a, pwm_b = data

        # Downsample if too many points
        max_points = 2000
        if len(t) > max_points:
            step = len(t) // max_points
            t = t[::step]
            current_a = current_a[::step]
            current_b = current_b[::step]
            pwm_a = pwm_a[::step]
            pwm_b = pwm_b[::step]

        self.current_a_curve.setData(t, current_a)
        self.current_b_curve.setData(t, current_b)
        self.pwm_a_curve.setData(t, pwm_a)
        self.pwm_b_curve.setData(t, pwm_b)

    def update_stats(self):
        """Update statistics display."""
        stats = self.buffer.get_stats(self.window_sec)

        if stats:
            self.stats_labels['current_a_min'].setText(str(stats['current_a_min']))
            self.stats_labels['current_a_max'].setText(str(stats['current_a_max']))
            self.stats_labels['current_a_rms'].setText(f"{stats['current_a_rms']:.1f}")
            self.stats_labels['current_b_min'].setText(str(stats['current_b_min']))
            self.stats_labels['current_b_max'].setText(str(stats['current_b_max']))
            self.stats_labels['current_b_rms'].setText(f"{stats['current_b_rms']:.1f}")
            self.stats_labels['pwm_a_min'].setText(str(stats['pwm_a_min']))
            self.stats_labels['pwm_a_max'].setText(str(stats['pwm_a_max']))
            self.stats_labels['pwm_b_min'].setText(str(stats['pwm_b_min']))
            self.stats_labels['pwm_b_max'].setText(str(stats['pwm_b_max']))

        self.frame_count_label.setText(str(self.total_frames))

        # Calculate frame rate
        if hasattr(self, '_last_frame_count'):
            rate = (self.total_frames - self._last_frame_count) * 2  # 500ms interval
            self.frame_rate_label.setText(f"{rate} Hz")
        self._last_frame_count = self.total_frames

        # Sync errors
        self.sync_err_label.setText(str(self.reader.parser.sync_errors))

    def closeEvent(self, event):
        """Handle window close."""
        self.reader.disconnect()
        event.accept()


# ==============================================================================
# Main
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='M23 Current Streaming Viewer',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--port', '-p', type=str, default=None,
                        help='Serial port')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--window', '-w', type=float, default=1.0,
                        help='Time window in seconds (default: 1.0)')
    parser.add_argument('-P', '--interactive', action='store_true',
                        help='Interactive port selection')

    args = parser.parse_args()

    # Handle interactive port selection
    port = args.port
    if args.interactive and port is None:
        ports = serial.tools.list_ports.comports()
        if ports:
            print("\nAvailable ports:")
            for i, p in enumerate(ports):
                print(f"  {i + 1}. {p.device}")
            try:
                choice = input("\nSelect port number: ").strip()
                idx = int(choice) - 1
                if 0 <= idx < len(ports):
                    port = ports[idx].device
            except (ValueError, KeyboardInterrupt):
                pass

    # Create Qt application
    app = QApplication(sys.argv)

    # Create and show main window
    window = M23CurrentViewer(port=port, baud=args.baud, window_sec=args.window)
    window.show()

    # Handle Ctrl+C
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec() if QT_VERSION == 6 else app.exec_())


if __name__ == '__main__':
    main()
