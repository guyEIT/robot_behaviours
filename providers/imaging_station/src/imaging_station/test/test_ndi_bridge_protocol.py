"""Unit tests for the AVB1 bridge protocol parser. Pure stdlib, no ROS deps.

Run via: colcon test --packages-select imaging_station
"""

import io
import os
import sys
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from imaging_station.ndi_bridge_protocol import (  # noqa: E402
    FOURCC_BGRX,
    HEADER_STRUCT,
    Packet,
    PacketType,
    fourcc_to_string,
    pack_header,
    read_packet,
)


class TestHeader(unittest.TestCase):
    def test_size_is_64(self):
        self.assertEqual(HEADER_STRUCT.size, 64)


class TestRoundTrip(unittest.TestCase):
    def _round_trip(self, packet: Packet) -> Packet:
        buf = pack_header(packet) + packet.payload
        stream = io.BytesIO(buf)
        result = read_packet(stream)
        assert result is not None
        return result

    def test_status_packet(self):
        original = Packet(
            packet_type=PacketType.STATUS,
            payload=b"hello world",
            timestamp=42,
        )
        decoded = self._round_trip(original)
        self.assertEqual(decoded.packet_type, PacketType.STATUS)
        self.assertEqual(decoded.text, "hello world")
        self.assertEqual(decoded.timestamp, 42)

    def test_video_packet(self):
        width, height = 4, 2
        payload = bytes(range(width * height * 4))  # 32 bytes BGRX
        original = Packet(
            packet_type=PacketType.VIDEO,
            payload=payload,
            timestamp=1234567,
            width=width,
            height=height,
            stride=width * 4,
            fourcc=FOURCC_BGRX,
            fps_n=30,
            fps_d=1,
        )
        decoded = self._round_trip(original)
        self.assertEqual(decoded.packet_type, PacketType.VIDEO)
        self.assertEqual(decoded.width, width)
        self.assertEqual(decoded.height, height)
        self.assertEqual(decoded.stride, width * 4)
        self.assertEqual(decoded.fourcc, FOURCC_BGRX)
        self.assertEqual(decoded.fps_n, 30)
        self.assertEqual(decoded.fps_d, 1)
        self.assertEqual(decoded.payload, payload)

    def test_eos_with_empty_payload(self):
        original = Packet(packet_type=PacketType.EOS, payload=b"", timestamp=0)
        decoded = self._round_trip(original)
        self.assertEqual(decoded.packet_type, PacketType.EOS)
        self.assertEqual(decoded.payload, b"")

    def test_truncated_stream_returns_none(self):
        # Empty stream → clean EOF → None
        self.assertIsNone(read_packet(io.BytesIO(b"")))

    def test_partial_header_raises_or_returns_none(self):
        # Half a header is unexpected EOF; treat as None per read_packet contract.
        partial = io.BytesIO(b"\x00" * 10)
        self.assertIsNone(read_packet(partial))

    def test_bad_magic_raises(self):
        bogus = b"\x00" * HEADER_STRUCT.size
        with self.assertRaises(ValueError):
            read_packet(io.BytesIO(bogus))


class TestFourCC(unittest.TestCase):
    def test_bgrx_round_trip(self):
        self.assertEqual(fourcc_to_string(FOURCC_BGRX), "BGRX")


class TestBgrxToBgrSlice(unittest.TestCase):
    """Sanity-check the numpy slice that ndi_node uses to drop the X channel."""

    def test_drops_alpha(self):
        import numpy as np

        width, height = 3, 2
        # Construct a BGRX frame where each pixel is (B, G, R, X) = (10, 20, 30, 99)
        bgrx = np.tile(
            np.array([10, 20, 30, 99], dtype=np.uint8), (height, width, 1)
        ).reshape(height, width, 4)
        bgr = np.ascontiguousarray(bgrx[:, :, :3])
        self.assertEqual(bgr.shape, (height, width, 3))
        self.assertTrue((bgr[:, :, 0] == 10).all())
        self.assertTrue((bgr[:, :, 1] == 20).all())
        self.assertTrue((bgr[:, :, 2] == 30).all())
        self.assertEqual(bgr.tobytes()[:3], bytes((10, 20, 30)))


if __name__ == "__main__":
    unittest.main()
