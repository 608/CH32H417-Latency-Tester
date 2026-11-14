#!/usr/bin/env python3
"""
lat_proto.py - Shared CDC communication protocol for the tester.

Contents:
  - CRC-16/CCITT-FALSE, COBS encoding/decoding
  - Frame building/parsing and FrameReader
  - HELLO payload building/parsing
  - TesterLink: synchronous HELLO and command/event transport over serial

Only TesterLink touches hardware; everything else is pure computation.
pyserial is imported lazily inside TesterLink.open().
"""
import binascii
import secrets
import struct
import time
from dataclasses import dataclass

PROTOCOL_VERSION = 1

# Message types
TYPE_HELLO = 0x01
TYPE_GET_STATUS = 0x02
TYPE_GET_OBJECT = 0x04
TYPE_REENUMERATE = 0x09
TYPE_ENTER_BOOTLOADER = 0x15
TYPE_READ_OBJECT = 0x16
RESPONSE_BIT = 0x80

# Event types
TYPE_DEVICE_ATTACHED = 0x41
TYPE_DEVICE_DETACHED = 0x42
TYPE_ENUM_PROGRESS = 0x43
TYPE_DEVICE_READY = 0x44
EVENT_TYPE_MIN = 0x40
EVENT_TYPE_MAX = 0x4C

# Response status codes
STATUS_OK = 0x00
STATUS_UNKNOWN_COMMAND = 0x01
STATUS_BAD_LENGTH = 0x02
STATUS_BAD_PARAM = 0x03
STATUS_BUSY = 0x04
STATUS_NO_DEVICE = 0x05
STATUS_NOT_READY = 0x06
STATUS_UNSUPPORTED = 0x07
STATUS_INTERNAL = 0x08
STATUS_SEQ_CONFLICT = 0x09
STATUS_STALE_OBJECT = 0x0A

STATUS_NAMES = {
    STATUS_OK: "OK",
    STATUS_UNKNOWN_COMMAND: "UNKNOWN_COMMAND",
    STATUS_BAD_LENGTH: "BAD_LENGTH",
    STATUS_BAD_PARAM: "BAD_PARAM",
    STATUS_BUSY: "BUSY",
    STATUS_NO_DEVICE: "NO_DEVICE",
    STATUS_NOT_READY: "NOT_READY",
    STATUS_UNSUPPORTED: "UNSUPPORTED",
    STATUS_INTERNAL: "INTERNAL",
    STATUS_SEQ_CONFLICT: "SEQ_CONFLICT",
    STATUS_STALE_OBJECT: "STALE_OBJECT",
}

HELLO_MAGIC = 0x5454414C  # 'LATT'
BOOT_CONFIRM = 0x424F4F54  # 'BOOT'

# Frame sizes
HEADER_FORMAT = "<BBBBHHI"
HEADER_SIZE = 12
CRC_SIZE = 2
MAX_PAYLOAD = 1152
MAX_DECODED_SIZE = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE  # 1166
MAX_WIRE_SIZE = 1172  # 1171 COBS-encoded bytes + 0x00 delimiter

HELLO_REQUEST_SIZE = 12
HELLO_RESPONSE_FORMAT = "<IBBHH12s16sIHHIQI"
HELLO_RESPONSE_SIZE = 62

# Per-read serial timeout in seconds for TesterLink
READ_TIMEOUT_S = 0.05

OBJECT_MAX_LENGTH = 64 * 1024
OBJECT_READ_CHUNK = 1024
OBJECT_STALE_RESTARTS = 3


class ProtocolError(RuntimeError):
    """Raised when a tester response violates the protocol."""


class ProtocolTimeout(ProtocolError):
    """Raised when no valid response arrives after all retries."""


def status_name(status: int) -> str:
    return STATUS_NAMES.get(status, f"0x{status:02X}")


def crc16_ccitt_false(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, no XOR)."""
    # binascii.crc_hqx computes unreflected CRC-16 with poly 0x1021 and the given initial value.
    return binascii.crc_hqx(data, 0xFFFF)


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode without the trailing 0x00 delimiter.

    If the input ends on an exact 254-byte non-zero run, no empty 0x01
    block is appended (matching the firmware's shortest-form encoding).
    """
    out = bytearray(b"\x00")
    code_index = 0
    code = 1
    for index, value in enumerate(data):
        if value == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue
        out.append(value)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            if index + 1 == len(data):
                return bytes(out)
            code_index = len(out)
            out.append(0)
            code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """COBS-decode a frame (excluding the 0x00 delimiter).

    Raises ValueError on empty input, embedded 0x00, or overrun.
    """
    if not data:
        raise ValueError("COBS: empty input")
    out = bytearray()
    index = 0
    size = len(data)
    while index < size:
        code = data[index]
        if code == 0:
            raise ValueError("COBS: zero byte in encoded data")
        end = index + code
        if end > size:
            raise ValueError("COBS: code byte overruns the input")
        block = data[index + 1:end]
        if 0 in block:
            raise ValueError("COBS: zero byte in encoded data")
        out += block
        index = end
        if code != 0xFF and index < size:
            out.append(0)
    return bytes(out)


@dataclass(frozen=True)
class Frame:
    version: int
    type: int
    flags: int
    status: int
    seq: int
    length: int
    session_id: int
    payload: bytes


def build_frame(type: int, seq: int, session_id: int, payload: bytes = b"",
                status: int = 0) -> bytes:
    """Build a decoded frame (header, payload, CRC-16)."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload is too long: {len(payload)} > {MAX_PAYLOAD}")
    body = struct.pack(HEADER_FORMAT, PROTOCOL_VERSION, type, 0, status,
                       seq, len(payload), session_id) + bytes(payload)
    return body + struct.pack("<H", crc16_ccitt_false(body))


def encode_wire(frame: bytes) -> bytes:
    """Encode a decoded frame into wire format (COBS + 0x00 delimiter)."""
    return cobs_encode(frame) + b"\x00"


def parse_frame(decoded: bytes) -> Frame:
    """Parse a decoded frame; raises ValueError on length or CRC mismatch."""
    if len(decoded) < HEADER_SIZE + CRC_SIZE:
        raise ValueError(f"frame is too short: {len(decoded)}")
    version, type_, flags, status, seq, length, session_id = struct.unpack_from(
        HEADER_FORMAT, decoded, 0)
    if length > MAX_PAYLOAD:
        raise ValueError(f"payload length is too large: {length}")
    if len(decoded) != HEADER_SIZE + length + CRC_SIZE:
        raise ValueError(
            f"frame length mismatch: {len(decoded)} != {HEADER_SIZE + length + CRC_SIZE}")
    body_end = HEADER_SIZE + length
    stored_crc = struct.unpack_from("<H", decoded, body_end)[0]
    calculated_crc = crc16_ccitt_false(decoded[:body_end])
    if stored_crc != calculated_crc:
        raise ValueError(
            f"CRC mismatch: stored=0x{stored_crc:04X}, calculated=0x{calculated_crc:04X}")
    return Frame(version, type_, flags, status, seq, length, session_id,
                 bytes(decoded[HEADER_SIZE:body_end]))


class FrameReader:
    """Split incoming bytes on 0x00 and yield valid decoded frames.

    Frames failing COBS, length, or CRC are silently dropped. If bytes exceed
    MAX_WIRE_SIZE without 0x00, discard until the next 0x00.
    """

    def __init__(self):
        self._buffer = bytearray()
        self._discarding = False

    def feed(self, data: bytes) -> list[Frame]:
        frames = []
        parts = bytes(data).split(b"\x00")
        for index, part in enumerate(parts):
            if not self._discarding:
                self._buffer += part
                if len(self._buffer) > MAX_WIRE_SIZE:
                    self._buffer.clear()
                    self._discarding = True
            if index == len(parts) - 1:
                break  # Trailing chunk has not seen 0x00 yet
            if not self._discarding:
                try:
                    frames.append(parse_frame(cobs_decode(bytes(self._buffer))))
                except ValueError:
                    pass
            self._buffer.clear()
            self._discarding = False
        return frames


@dataclass(frozen=True)
class HelloInfo:
    magic: int
    protocol_version: int
    fw_major: int
    fw_minor: int
    build: str
    uuid: bytes
    capabilities: int
    max_payload: int
    max_poll_raw_count: int
    host_nonce_echo: int
    session_id: int


def decode_build(raw: bytes) -> str:
    """Format the 12-byte build timestamp as a string, escaping non-ASCII as \\xNN."""
    return raw.decode("ascii", errors="backslashreplace").rstrip("\x00")


def hello_request_payload(nonce: int) -> bytes:
    return struct.pack("<B3sQ", PROTOCOL_VERSION, b"\0\0\0", nonce)


def parse_hello_response(payload: bytes) -> HelloInfo:
    if len(payload) != HELLO_RESPONSE_SIZE:
        raise ValueError(
            f"HELLO response length is {len(payload)}, expected {HELLO_RESPONSE_SIZE}")
    (magic, protocol_version, _reserved0, fw_major, fw_minor, build, uuid,
     capabilities, max_payload, _reserved1, max_poll_raw_count, host_nonce_echo,
     session_id) = struct.unpack(HELLO_RESPONSE_FORMAT, payload)
    return HelloInfo(magic, protocol_version, fw_major, fw_minor, decode_build(build),
                     uuid, capabilities, max_payload, max_poll_raw_count,
                     host_nonce_echo, session_id)


class TesterLink:
    """Synchronous HELLO and command/event transport over the tester CDC port.

    port must implement pyserial Serial methods (read, write, in_waiting,
    reset_input_buffer, close); read timeout is set to READ_TIMEOUT_S.
    Only one command is in flight at a time, retried with the same seq/payload
    on timeout. Events for the current session are queued; frames with a
    mismatched session_id are discarded.
    """

    def __init__(self, port):
        self.port = port
        self.port.timeout = READ_TIMEOUT_S
        self.reader = FrameReader()
        self.session_id = 0
        self.hello_info = None
        self._seq = 0
        self._events = []

    @classmethod
    def open(cls, port_name: str) -> "TesterLink":
        import serial
        return cls(serial.Serial(port_name, timeout=READ_TIMEOUT_S, write_timeout=1.0))

    def close(self) -> None:
        # Ignore errors when closing a port that disappeared after a reboot.
        try:
            self.port.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def _transact(self, wire: bytes, accept, timeout: float, retries: int) -> Frame:
        """Send wire and wait for a frame where accept(frame) is true, retrying on timeout."""
        pending_events = []
        for _attempt in range(retries + 1):
            self.port.write(wire)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                data = self.port.read(max(1, self.port.in_waiting))
                accepted = None
                for frame in self.reader.feed(data):
                    if frame.version != PROTOCOL_VERSION:
                        continue
                    if (self.session_id != 0 and frame.session_id == self.session_id
                            and EVENT_TYPE_MIN <= frame.type <= EVENT_TYPE_MAX):
                        self._events.append(frame)
                    elif (self.session_id == 0
                          and EVENT_TYPE_MIN <= frame.type <= EVENT_TYPE_MAX):
                        pending_events.append(frame)
                    if accepted is None and accept(frame):
                        accepted = frame
                # Queue all events from the same read() even if they follow the response.
                if accepted is not None:
                    if self.session_id == 0 and accepted.session_id != 0:
                        self._events.extend(
                            event for event in pending_events
                            if event.session_id == accepted.session_id)
                    return accepted
        raise ProtocolTimeout(f"no response after {retries + 1} attempts")

    def hello(self, timeout: float = 1.0, retries: int = 3) -> HelloInfo:
        """Send HELLO with a fresh random nonce to start a session."""
        nonce = secrets.randbits(64)
        seq = self._next_seq()
        wire = encode_wire(build_frame(TYPE_HELLO, seq, 0, hello_request_payload(nonce)))
        self.port.reset_input_buffer()
        self.reader = FrameReader()
        self.session_id = 0
        self.hello_info = None
        self._events.clear()

        def accept(frame: Frame) -> bool:
            if frame.type != TYPE_HELLO | RESPONSE_BIT or frame.seq != seq:
                return False
            # Discard stale HELLO responses from earlier attempts.
            if frame.status == STATUS_OK and len(frame.payload) == HELLO_RESPONSE_SIZE:
                return parse_hello_response(frame.payload).host_nonce_echo == nonce
            return True

        frame = self._transact(wire, accept, timeout, retries)
        if frame.status != STATUS_OK:
            raise ProtocolError(f"HELLO failed: {status_name(frame.status)}")
        try:
            info = parse_hello_response(frame.payload)
        except ValueError as error:
            raise ProtocolError(str(error)) from error
        if info.magic != HELLO_MAGIC:
            raise ProtocolError(f"HELLO magic is 0x{info.magic:08X}, expected 0x{HELLO_MAGIC:08X}")
        if info.protocol_version != PROTOCOL_VERSION:
            raise ProtocolError(f"unsupported protocol version: {info.protocol_version}")
        if info.session_id == 0 or frame.session_id != info.session_id:
            raise ProtocolError(
                f"invalid session id: header=0x{frame.session_id:08X}, "
                f"payload=0x{info.session_id:08X}")
        self.session_id = info.session_id
        self.hello_info = info
        return info

    def command(self, type: int, payload: bytes = b"", timeout: float = 1.0,
                retries: int = 3) -> tuple[int, bytes]:
        """Send a command and return (status, response_payload). Requires hello() first."""
        if self.session_id == 0:
            raise ProtocolError("HELLO has not been completed")
        seq = self._next_seq()
        session_id = self.session_id
        wire = encode_wire(build_frame(type, seq, session_id, payload))

        def accept(frame: Frame) -> bool:
            return (frame.type == type | RESPONSE_BIT and frame.seq == seq
                    and frame.session_id == session_id)

        frame = self._transact(wire, accept, timeout, retries)
        return frame.status, frame.payload

    def wait_event(self, types, timeout: float, predicate=None) -> Frame:
        """Wait for an event of the given type; non-matching events remain queued."""
        if self.session_id == 0:
            raise ProtocolError("HELLO has not been completed")
        wanted = {types} if isinstance(types, int) else set(types)
        if not wanted:
            raise ValueError("types must not be empty")

        deadline = time.monotonic() + timeout
        while True:
            for index, event in enumerate(self._events):
                if event.type in wanted and (predicate is None or predicate(event)):
                    return self._events.pop(index)
            if time.monotonic() >= deadline:
                names = ", ".join(f"0x{type_:02X}" for type_ in sorted(wanted))
                raise ProtocolTimeout(f"no event ({names}) before timeout")
            data = self.port.read(max(1, self.port.in_waiting))
            for frame in self.reader.feed(data):
                if (frame.version == PROTOCOL_VERSION and frame.session_id == self.session_id
                        and EVENT_TYPE_MIN <= frame.type <= EVENT_TYPE_MAX):
                    self._events.append(frame)

    def read_object(self, kind: int, index: int, source_id: int,
                    source_id_provider=None, chunk_size: int = OBJECT_READ_CHUNK) -> bytes:
        """Lock an object with GET_OBJECT and read it completely with READ_OBJECT."""
        if not 0 <= kind <= 0xFF or not 0 <= index <= 0xFF:
            raise ValueError("kind and index must fit in u8")
        if not 1 <= chunk_size <= OBJECT_READ_CHUNK:
            raise ValueError(f"chunk_size must be 1..{OBJECT_READ_CHUNK}")
        current_source_id = source_id

        for stale_attempt in range(OBJECT_STALE_RESTARTS + 1):
            if not 0 <= current_source_id <= 0xFFFFFFFF:
                raise ValueError("source_id must fit in u32")
            status, payload = self.command(
                TYPE_GET_OBJECT, struct.pack("<BBHI", kind, index, 0, current_source_id))
            stale = status == STATUS_STALE_OBJECT
            if not stale:
                if status != STATUS_OK:
                    raise ProtocolError(f"GET_OBJECT failed: {status_name(status)}")
                if len(payload) != 16:
                    raise ProtocolError(
                        f"GET_OBJECT response length is {len(payload)}, expected 16")
                object_id, got_kind, got_index, got_source_id, total_length, crc32 = \
                    struct.unpack("<HBBIII", payload)
                if (got_kind, got_index, got_source_id) != (kind, index, current_source_id):
                    raise ProtocolError("GET_OBJECT response does not match the request")
                if total_length > OBJECT_MAX_LENGTH:
                    raise ProtocolError(
                        f"object length is {total_length}, maximum is {OBJECT_MAX_LENGTH}")

                data = bytearray()
                while len(data) < total_length:
                    offset = len(data)
                    status, payload = self.command(
                        TYPE_READ_OBJECT, struct.pack("<HHI", object_id, chunk_size, offset))
                    if status == STATUS_STALE_OBJECT:
                        stale = True
                        break
                    if status != STATUS_OK:
                        raise ProtocolError(f"READ_OBJECT failed: {status_name(status)}")
                    if len(payload) < 16:
                        raise ProtocolError(
                            f"READ_OBJECT response is too short: {len(payload)}")
                    (got_object_id, data_length, got_offset, got_total_length,
                     got_crc32) = struct.unpack_from("<HHIII", payload)
                    expected_length = min(chunk_size, total_length - offset)
                    if got_object_id != object_id:
                        raise ProtocolError("READ_OBJECT object_id changed")
                    if got_offset != offset:
                        raise ProtocolError(
                            f"READ_OBJECT offset is {got_offset}, expected {offset}")
                    if data_length != expected_length:
                        raise ProtocolError(
                            f"READ_OBJECT data_length is {data_length}, expected {expected_length}")
                    if got_total_length != total_length:
                        raise ProtocolError("READ_OBJECT total_length changed")
                    if got_crc32 != crc32:
                        raise ProtocolError("READ_OBJECT CRC-32 changed")
                    if len(payload) != 16 + data_length:
                        raise ProtocolError(
                            f"READ_OBJECT response length is {len(payload)}, "
                            f"expected {16 + data_length}")
                    data += payload[16:]

                if not stale:
                    calculated_crc32 = binascii.crc32(data) & 0xFFFFFFFF
                    if calculated_crc32 != crc32:
                        raise ProtocolError(
                            f"object CRC-32 mismatch: stored=0x{crc32:08X}, "
                            f"calculated=0x{calculated_crc32:08X}")
                    return bytes(data)

            if stale_attempt == OBJECT_STALE_RESTARTS:
                break
            if source_id_provider is not None:
                current_source_id = source_id_provider()

        raise ProtocolError(
            f"object stayed stale after {OBJECT_STALE_RESTARTS + 1} attempts")

    def reenumerate(self) -> None:
        """Start Windows-style re-enumeration; progress is delivered via wait_event()."""
        status, payload = self.command(TYPE_REENUMERATE)
        if status != STATUS_OK:
            raise ProtocolError(f"REENUMERATE failed: {status_name(status)}")
        if payload:
            raise ProtocolError(
                f"REENUMERATE response length is {len(payload)}, expected 0")
