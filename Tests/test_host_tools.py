"""Check Linux host framing and decoding without opening a hardware bus."""
import struct
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from poweroff_ec import request_power_off
from read_ec import decode_telemetry, read_telemetry


class Message:
    def __init__(self, address, is_read, data):
        self.address = address
        self.is_read = is_read
        self.data = bytearray(data)

    def __bytes__(self):
        return bytes(self.data)

    @staticmethod
    def write(address, data):
        return Message(address, False, data)

    @staticmethod
    def read(address, length):
        return Message(address, True, bytes(length))


def snapshot(flags=0x08):
    data = bytearray(28)
    struct.pack_into("<BBHBB", data, 0, 1, 28, 0, flags, 0)
    return data


class FakeBus:
    def __init__(self, data=None, fail_read=False):
        self.data = snapshot() if data is None else data
        self.calls = []
        self.fail_read = fail_read

    def i2c_rdwr(self, *messages):
        self.calls.append(tuple((msg.address, msg.is_read, bytes(msg)) for msg in messages))
        for msg in messages:
            if msg.is_read:
                if self.fail_read:
                    raise OSError("mock I2C read failure")
                msg.data[:] = self.data


class HostToolsTests(unittest.TestCase):
    def setUp(self):
        self.fake_smbus = patch.dict(sys.modules, {"smbus2": types.SimpleNamespace(i2c_msg=Message)})
        self.fake_smbus.start()
        self.addCleanup(self.fake_smbus.stop)

    def test_power_off_frame_is_single_standalone_write(self):
        for seconds, expected in [(0, b"\x20\xa5\x00\x00"),
                                  (5, b"\x20\xa5\x05\x00"),
                                  (300, b"\x20\xa5\x2c\x01"),
                                  (65535, b"\x20\xa5\xff\xff")]:
            with self.subTest(seconds=seconds):
                bus = FakeBus()
                request_power_off(bus, seconds)
                self.assertEqual(len(bus.calls), 2)
                # Preflight is a separate combined telemetry read.
                self.assertEqual(bus.calls[0], ((0x42, False, b"\x00"),
                                               (0x42, True, bytes(28))))
                # The control transfer contains only one write message, so
                # i2c_rdwr ends it with STOP, without a repeated START.
                self.assertEqual(bus.calls[1], ((0x42, False, expected),))

    def test_selected_address_is_used_for_read_and_write(self):
        bus = FakeBus()
        request_power_off(bus, 1, address=0x43)
        self.assertTrue(all(address == 0x43 for call in bus.calls for address, _, _ in call))

    def test_out_of_range_delay_does_not_access_bus(self):
        for seconds in (-1, 65536):
            with self.subTest(seconds=seconds):
                bus = FakeBus()
                with self.assertRaises(ValueError):
                    request_power_off(bus, seconds)
                self.assertEqual(bus.calls, [])

    def test_legacy_firmware_is_read_but_never_sent_control(self):
        bus = FakeBus(snapshot(flags=0x07))
        with self.assertRaisesRegex(ValueError, "does not support"):
            request_power_off(bus, 5)
        self.assertEqual(len(bus.calls), 1)

    def test_failed_or_unknown_telemetry_prevents_control_write(self):
        malformed = snapshot()
        malformed[0] = 2
        for bus, error in [(FakeBus(fail_read=True), OSError),
                           (FakeBus(malformed), ValueError),
                           (FakeBus(bytes(27)), ValueError)]:
            with self.subTest(error=error):
                with self.assertRaises(error):
                    request_power_off(bus, 5)
                self.assertEqual(len(bus.calls), 1)

    def test_new_flags_preserve_battery_decoding(self):
        data = snapshot(flags=0x1F)
        struct.pack_into("<H", data, 2, (1 << 0) | (1 << 1) | (1 << 7))
        data[5] = 3
        struct.pack_into("<Hh", data, 6, 3900, -500)
        struct.pack_into("<h", data, 20, -480)
        struct.pack_into("<I", data, 24, 123)
        result = read_telemetry(FakeBus(data))
        self.assertTrue(result["power_off_supported"])
        self.assertTrue(result["power_off_pending"])
        self.assertTrue(result["load_on"])
        self.assertEqual(result["voltage_mv"], 3900)
        self.assertEqual(result["current_ma"], -500)
        self.assertEqual(result["average_current_ma"], -480)
        self.assertEqual(result["sample_sequence"], 123)
        self.assertEqual(result["battery_state"], "discharging")
        self.assertIsNone(result["soc_percent"])
        self.assertIsNone(result["time_to_full_min"])
        self.assertIsNone(result["time_to_empty_min"])

    def test_legacy_flags_decode_without_power_off_support(self):
        result = decode_telemetry(snapshot(flags=0x07))
        self.assertFalse(result["power_off_supported"])
        self.assertFalse(result["power_off_pending"])


if __name__ == "__main__":
    unittest.main()
