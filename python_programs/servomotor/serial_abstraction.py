"""
Serial Port Abstraction Layer

Provides a unified interface for serial communication across different platforms
(standard Python with pyserial and MicroPython with machine.UART).
"""

import time
from .platform_detect import is_micropython


class SerialPort:
    """
    Base class for serial port operations.
    Defines the interface that all platform-specific implementations must follow.
    """
    
    def __init__(self, port=None, baudrate=230400, timeout=0.1):
        """
        Initialize serial port.
        
        Args:
            port: Serial port identifier (device name for standard Python, ignored for MicroPython)
            baudrate: Baud rate for communication (default: 230400)
            timeout: Read timeout in seconds (default: 0.1)
        """
        raise NotImplementedError("Subclass must implement __init__")
    
    def write(self, data):
        """
        Write data to the serial port.
        
        Args:
            data: Bytes to write
            
        Returns:
            Number of bytes written
        """
        raise NotImplementedError("Subclass must implement write")
    
    def read(self, size=1):
        """
        Read specified number of bytes from serial port.
        
        Args:
            size: Number of bytes to read
            
        Returns:
            bytes: Data read from serial port
        """
        raise NotImplementedError("Subclass must implement read")
    
    def reset_input_buffer(self):
        """Clear the input buffer."""
        raise NotImplementedError("Subclass must implement reset_input_buffer")
    
    def close(self):
        """Close the serial port."""
        raise NotImplementedError("Subclass must implement close")
    
    @property
    def name(self):
        """Get the name/identifier of the serial port."""
        raise NotImplementedError("Subclass must implement name property")


class StandardSerial(SerialPort):
    """
    Serial port implementation for standard Python using pyserial.
    Used on Mac, PC, Raspberry Pi, etc.
    """
    
    def __init__(self, port=None, baudrate=230400, timeout=0.1):
        """
        Initialize serial port using pyserial.
        
        Args:
            port: Serial port device name (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Baud rate for communication
            timeout: Read timeout in seconds
        """
        from .vendor import serial
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self._name = port if port else self.ser.name
    
    def write(self, data):
        """Write data to the serial port."""
        return self.ser.write(data)
    
    def read(self, size=1):
        """Read specified number of bytes."""
        return self.ser.read(size)
    
    def reset_input_buffer(self):
        """Clear the input buffer."""
        self.ser.reset_input_buffer()
    
    def close(self):
        """Close the serial port."""
        self.ser.close()
    
    @property
    def name(self):
        """Get the serial port name."""
        return self._name


class MicroPythonSerial(SerialPort):
    """
    Serial port implementation for MicroPython using machine.UART.
    Designed for ESP32-S3 with TX=Pin(4), RX=Pin(5).
    """
    
    def __init__(self, port=None, baudrate=230400, timeout=0.1):
        """
        Initialize UART for MicroPython.
        
        Args:
            port: Ignored for MicroPython (uses UART(1) by default)
            baudrate: Baud rate for communication
            timeout: Read timeout in seconds
        """
        from machine import UART, Pin
        
        # Default configuration for ESP32-S3
        # UART(1) is the secondary UART port
        # TX=Pin(4), RX=Pin(5) as specified in requirements
        self.uart = UART(
            1,                          # UART(1) is the secondary port
            baudrate=baudrate,
            bits=8,
            parity=None,
            stop=1,
            tx=Pin(4),                  # TX pin
            rx=Pin(5),                  # RX pin
            timeout=int(timeout * 1000) # Convert to milliseconds
        )
        
        self.timeout = timeout
        self._name = "UART(1) [TX=Pin(4), RX=Pin(5)]"
    
    def write(self, data):
        """
        Write data to the UART.
        
        Args:
            data: Bytes to write
            
        Returns:
            Number of bytes written (or None if not available)
        """
        return self.uart.write(data)
    
    def read(self, size=1):
        """
        Read specified number of bytes from UART.
        
        Args:
            size: Number of bytes to read
            
        Returns:
            bytes: Data read (may be less than requested if timeout occurs)
        """
        # MicroPython UART.read() may return None if no data
        data = self.uart.read(size)
        return data if data is not None else b''
    
    def reset_input_buffer(self):
        """
        Clear the input buffer.
        
        Note: MicroPython doesn't have a direct equivalent to reset_input_buffer,
        so we read and discard all available data.
        """
        while self.uart.any():
            self.uart.read(self.uart.any())
    
    def close(self):
        """
        Close/deinitialize the UART.
        
        Note: MicroPython UART.deinit() releases the pins.
        """
        try:
            self.uart.deinit()
        except:
            # Some MicroPython implementations might not have deinit
            pass
    
    @property
    def name(self):
        """Get the UART identifier."""
        return self._name


def create_serial_port(port=None, baudrate=230400, timeout=0.1):
    """
    Factory function to create the appropriate serial port instance
    based on the current platform.

    Args:
        port: Serial port identifier (used only on standard Python)
        baudrate: Baud rate for communication (default: 230400)
        timeout: Read timeout in seconds (default: 0.1)

    Returns:
        SerialPort: Platform-appropriate serial port instance
    """
    if is_micropython():
        return MicroPythonSerial(port, baudrate, timeout)
    else:
        return StandardSerial(port, baudrate, timeout)


def open_serial_port(port=None, baudrate=230400, timeout=0.1):
    """
    Open the platform-appropriate serial port.

    On standard Python, performs the same device-name resolution that
    ``serial_functions.open_serial_port`` does (saved default, interactive
    menu when needed) and wraps the resulting pyserial port in a
    ``StandardSerial`` adapter so the rest of the library only sees the
    abstract ``SerialPort`` interface.

    On MicroPython, the ``port`` argument is ignored and a
    ``MicroPythonSerial`` is returned (UART(1), TX=Pin(4), RX=Pin(5)).
    """
    if is_micropython():
        return MicroPythonSerial(port, baudrate, timeout)

    from . import serial_functions
    pyserial_port = serial_functions.open_serial_port(port, baudrate, timeout)
    adapter = StandardSerial.__new__(StandardSerial)
    adapter.ser = pyserial_port
    adapter._name = pyserial_port.name
    return adapter