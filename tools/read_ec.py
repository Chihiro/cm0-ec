#!/usr/bin/env python3
"""Read one EC battery snapshot on a Linux I2C master (requires smbus2)."""
import argparse
import json
import struct

ADDRESS = 0x42
REGISTER_COUNT = 28
BATTERY_STATES = {0: "unknown", 1: "idle", 2: "charging", 3: "discharging", 4: "full"}


def decode_telemetry(data):
    if len(data) != REGISTER_COUNT:
        raise ValueError(f"Expected {REGISTER_COUNT} bytes, received {len(data)}")
    version, length, valid, flags, state = struct.unpack_from("<BBHBB", data)
    if version != 1 or length != REGISTER_COUNT:
        raise ValueError(f"Unsupported EC protocol: version={version}, length={length}")

    def field(offset, valid_bit, signed=False):
        if not valid & (1 << valid_bit):
            return None
        return struct.unpack_from("<h" if signed else "<H", data, offset)[0]

    return {
        "protocol_version": version,
        "sample_sequence": struct.unpack_from("<I", data, 24)[0],
        "gauge_normal": bool(flags & 4),
        "vbus_present": bool(flags & 1),
        "load_on": bool(flags & 2),
        "power_off_supported": bool(flags & 8),
        "power_off_pending": bool(flags & 16),
        "battery_state": BATTERY_STATES.get(state, "unknown"),
        "voltage_mv": field(6, 0),
        "current_ma": field(8, 1, signed=True),
        "soc_percent": field(10, 2),
        "time_to_full_min": field(12, 3),
        "time_to_empty_min": field(14, 4),
        "remaining_capacity_mah": field(16, 5),
        "full_charge_capacity_mah": field(18, 6),
        "average_current_ma": field(20, 7, signed=True),
        "battery_status": field(22, 8),
    }


def read_telemetry(bus, address=ADDRESS):
    from smbus2 import i2c_msg

    offset = i2c_msg.write(address, [0x00])
    response = i2c_msg.read(address, REGISTER_COUNT)
    bus.i2c_rdwr(offset, response)  # Combined write + repeated START + read.
    return decode_telemetry(bytes(response))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1, help="Linux I2C bus number (default: 1)")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=ADDRESS)
    args = parser.parse_args()
    if not 0x08 <= args.address <= 0x77:
        parser.error("address must be an unshifted 7-bit address from 0x08 to 0x77")
    try:
        from smbus2 import SMBus

        with SMBus(args.bus) as bus:
            result = read_telemetry(bus, args.address)
    except (ImportError, OSError, ValueError) as exc:
        parser.exit(1, f"EC read failed: {exc}\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
