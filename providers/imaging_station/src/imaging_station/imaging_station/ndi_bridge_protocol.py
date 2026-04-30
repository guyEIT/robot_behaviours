"""Wire format for the AVB1 packets emitted by ndi_av_bridge.

Mirrors experiment_collector/backend/app/services/ndi_sdk_bridge_protocol.py.
The bridge writes a 64-byte little-endian header followed by `payload_size`
bytes of payload, repeating until it sends an EOS packet or the socket
closes. Packet types: STATUS (text), VIDEO (frame), AUDIO (frame), EOS, ERROR.
"""
from __future__ import annotations

import enum
import socket
import struct
from dataclasses import dataclass
from typing import BinaryIO

MAGIC = 0x31425641  # "AVB1"
VERSION = 1
HEADER_STRUCT = struct.Struct("<IHHQqIIIIIIIIII")
assert HEADER_STRUCT.size == 64


def _fourcc(code: str) -> int:
    if len(code) != 4:
        raise ValueError(f"FourCC must be 4 characters, got {code!r}")
    return ord(code[0]) | (ord(code[1]) << 8) | (ord(code[2]) << 16) | (ord(code[3]) << 24)


FOURCC_UYVY = _fourcc("UYVY")
FOURCC_BGRA = _fourcc("BGRA")
FOURCC_BGRX = _fourcc("BGRX")
FOURCC_RGBA = _fourcc("RGBA")
FOURCC_RGBX = _fourcc("RGBX")


class PacketType(enum.IntEnum):
    STATUS = 1
    VIDEO = 2
    AUDIO = 3
    EOS = 4
    ERROR = 5


@dataclass(slots=True)
class Packet:
    packet_type: PacketType
    payload: bytes
    timestamp: int
    width: int = 0
    height: int = 0
    stride: int = 0
    fourcc: int = 0
    fps_n: int = 0
    fps_d: int = 0
    sample_rate: int = 0
    channels: int = 0
    samples: int = 0
    flags: int = 0

    @property
    def text(self) -> str:
        return self.payload.decode("utf-8", errors="replace")


def _read_exact(handle: socket.socket | BinaryIO, size: int) -> bytes:
    buf = bytearray(size)
    view = memoryview(buf)
    offset = 0
    while offset < size:
        if isinstance(handle, socket.socket):
            n = handle.recv_into(view[offset:])
        else:
            chunk = handle.read(size - offset)
            if not chunk:
                raise EOFError("unexpected EOF while reading AV bridge packet")
            n = len(chunk)
            view[offset : offset + n] = chunk
        if n == 0:
            raise EOFError("unexpected EOF while reading AV bridge packet")
        offset += n
    return bytes(buf)


def read_packet(handle: socket.socket | BinaryIO) -> Packet | None:
    """Read one packet from the bridge socket. Returns None on clean EOF."""
    try:
        raw_header = _read_exact(handle, HEADER_STRUCT.size)
    except EOFError:
        return None
    (
        magic,
        version,
        packet_type,
        payload_size,
        timestamp,
        width,
        height,
        stride,
        fourcc,
        fps_n,
        fps_d,
        sample_rate,
        channels,
        samples,
        flags,
    ) = HEADER_STRUCT.unpack(raw_header)
    if magic != MAGIC:
        raise ValueError(f"unexpected packet magic 0x{magic:08x}")
    if version != VERSION:
        raise ValueError(f"unexpected protocol version {version}")
    payload = _read_exact(handle, payload_size) if payload_size else b""
    return Packet(
        packet_type=PacketType(packet_type),
        payload=payload,
        timestamp=timestamp,
        width=width,
        height=height,
        stride=stride,
        fourcc=fourcc,
        fps_n=fps_n,
        fps_d=fps_d,
        sample_rate=sample_rate,
        channels=channels,
        samples=samples,
        flags=flags,
    )


def fourcc_to_string(value: int) -> str:
    return "".join(chr((value >> shift) & 0xFF) for shift in (0, 8, 16, 24))


def pack_header(packet: Packet) -> bytes:
    """Serialise the header for a Packet — used by tests that fake the bridge."""
    return HEADER_STRUCT.pack(
        MAGIC,
        VERSION,
        int(packet.packet_type),
        len(packet.payload),
        packet.timestamp,
        packet.width,
        packet.height,
        packet.stride,
        packet.fourcc,
        packet.fps_n,
        packet.fps_d,
        packet.sample_rate,
        packet.channels,
        packet.samples,
        packet.flags,
    )
