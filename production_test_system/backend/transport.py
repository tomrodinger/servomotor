"""Per-bus RS485 transport.

The servomotor library keeps its serial port in a *module global*
(``communication.ser``), so it can only ever talk to one bus at a time.  The
production rack has **three independent buses running in parallel**, so we
need one serial connection per bus with no shared global state.

This module provides exactly that while **reusing the library** for everything
that matters: the command table, the input encoder (``gather_inputs``), the
response decoder (``interpret_single_response``) and the CRC.  The only thing
implemented here is the small amount of packet *framing* and serial read/write
plumbing that the library otherwise hides behind its global ``ser`` — which has
to be per-port for three concurrent buses.

A :class:`Transport` is intentionally byte-level and synchronous (blocking
serial I/O); the bus workers each run it in their own thread.  A drop-in
:class:`SimulatedTransport` (see ``simulator.py``) lets the whole system run and
be tested with no hardware.
"""

from __future__ import annotations

import abc
import struct
from typing import Any, List, Optional

from . import bootstrap  # noqa: F401
import servomotor
from servomotor import communication as _c

# Re-export the library's exceptions so callers depend on one place.
TimeoutError = _c.TimeoutError
CommunicationError = _c.CommunicationError
PayloadError = _c.PayloadError
FatalError = _c.FatalError

ALL_ALIAS = _c.ALL_ALIAS                      # 255 broadcast
EXTENDED_ADDRESSING = _c.EXTENDED_ADDRESSING  # 254


def command_id_of(command: Any) -> int:
    """Resolve a command (int id or human string) to its numeric id."""
    if isinstance(command, int):
        return command
    cid = _c.get_command_id(command)
    if cid is None:
        raise ValueError("Unknown command: %r" % (command,))
    return cid


def _command_record(command_id: int) -> dict:
    for item in _c.registered_commands:
        if item["CommandEnum"] == command_id:
            return item
    raise ValueError("No registered command with id %d" % command_id)


class Transport(abc.ABC):
    """Abstract per-bus transport.

    ``transact`` sends one command and returns a list of *parsed* responses
    (each parsed response is the list of decoded output fields produced by the
    library's ``interpret_single_response``).  Broadcasts and commands with no
    output return an empty list.
    """

    @abc.abstractmethod
    def transact(self, address: int, command: Any, inputs: Optional[List[Any]] = None,
                 *, crc32_enabled: bool = True,
                 timeout: Optional[float] = None) -> List[List[Any]]:
        ...

    @abc.abstractmethod
    def flush_input(self) -> None:
        ...

    @abc.abstractmethod
    def close(self) -> None:
        ...

    @property
    @abc.abstractmethod
    def port(self) -> str:
        ...

    @property
    def is_simulated(self) -> bool:
        return False


class SerialTransport(Transport):
    """Real RS485 transport over a single pyserial port."""

    def __init__(self, port: str, baud: int = 230400, default_timeout: float = 1.2):
        import serial  # pyserial; imported lazily so the simulator path needs no hw libs
        self._port = port
        self._default_timeout = default_timeout
        self._ser = serial.Serial(port, baudrate=baud, timeout=default_timeout)

    @property
    def port(self) -> str:
        return self._port

    # -- framing (mirrors communication.send_command, but on our own port) --
    def _build_packet(self, address: int, command_id: int, payload: bytes,
                      crc32_enabled: bool) -> bytes:
        if address <= 255:
            address_part = struct.pack("<B", address)
        else:
            address_part = struct.pack("<BQ", EXTENDED_ADDRESSING, address)
        content = address_part + struct.pack("<B", command_id) + payload

        size = 1 + len(content)
        if crc32_enabled:
            size += 4
        if size > 127:
            size += 2
            packet = struct.pack("<BH", _c.encode_first_byte(127), size) + content
        else:
            packet = bytearray([_c.encode_first_byte(size)]) + content
        if crc32_enabled:
            packet = bytes(packet) + struct.pack("<I", servomotor.calculate_crc32(packet))
        return bytes(packet)

    def _read_one_response(self, crc32_enabled: bool) -> Optional[bytes]:
        """Read and validate one response frame; return its payload bytes.

        Returns the payload (with the leading error-code byte already checked
        and stripped) or raises TimeoutError / CommunicationError / FatalError.
        """
        first = self._ser.read(1)
        if len(first) != 1:
            raise TimeoutError("timeout reading first byte")
        if (first[0] & 0x01) != 0x01:
            raise CommunicationError("first byte 0x%02X has LSB not set" % first[0])
        packet_size = first[0] >> 1
        size_bytes = first
        if packet_size == 127:
            ext = self._ser.read(2)
            if len(ext) != 2:
                raise CommunicationError("could not read extended size")
            size_bytes = size_bytes + ext
            packet_size = ext[0] | (ext[1] << 8)
        remaining = packet_size - len(size_bytes)
        if remaining <= 0:
            raise CommunicationError("fewer bytes than expected in response")
        data = b""
        while len(data) < remaining:
            chunk = self._ser.read(remaining - len(data))
            if len(chunk) == 0:
                raise TimeoutError("timeout reading remaining bytes")
            data += chunk

        if data[0] == _c.RESPONSE_CHARACTER_CRC32_ENABLED:
            recv_crc = struct.unpack("<I", data[-4:])[0]
            calc_crc = servomotor.calculate_crc32(bytes(size_bytes) + data[:-4])
            if calc_crc != recv_crc:
                raise CommunicationError("CRC32 mismatch")
            payload = data[1:-4]
        elif data[0] == _c.RESPONSE_CHARACTER_CRC32_DISABLED:
            payload = data[1:]
        else:
            raise CommunicationError("bad response address byte %d" % data[0])

        if len(payload) == 0:
            return b""
        error_code = payload[0]
        if error_code != 0:
            raise FatalError(error_code)
        return payload[1:]

    def transact(self, address, command, inputs=None, *, crc32_enabled=True,
                 timeout=None):
        if inputs is None:
            inputs = []
        command_id = command_id_of(command)
        record = _command_record(command_id)
        payload = bytes(_c.gather_inputs(command_id, inputs, verbose=0))

        if timeout is not None and timeout != self._default_timeout:
            self._ser.timeout = timeout
        try:
            self._ser.write(self._build_packet(address, command_id, payload, crc32_enabled))

            # Decide whether to expect responses (mirrors send_command).
            is_detect = (command_id == command_id_of("Detect devices"))
            if address == ALL_ALIAS and not is_detect:
                return []
            if record["Output"] == []:
                return []

            multiple = record.get("MultipleResponses", False) or is_detect
            parsed_list: List[List[Any]] = []
            while True:
                try:
                    raw = self._read_one_response(crc32_enabled)
                except TimeoutError:
                    if address == ALL_ALIAS or multiple:
                        break
                    raise
                # A CommunicationError (CRC/framing) propagates: on a detect it
                # means responses collided on the half-duplex bus, and the caller
                # retries the whole pass (see detection.run_detect_pass).
                parsed_list.append(_c.interpret_single_response(command_id, raw, verbose=0))
                if not multiple:
                    break
            return parsed_list
        finally:
            if timeout is not None and timeout != self._default_timeout:
                self._ser.timeout = self._default_timeout

    def flush_input(self) -> None:
        self._ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass
