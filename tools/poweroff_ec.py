#!/usr/bin/env python3
"""Ask the EC to switch off the PA1 load after an explicit delay in seconds."""
import argparse
import struct

from read_ec import ADDRESS, read_telemetry

CONTROL_REGISTER = 0x20
POWER_OFF_COMMAND = 0xA5


def request_power_off(bus, delay_seconds, address=ADDRESS):
    if not 0 <= delay_seconds <= 65535:
        raise ValueError("delay_seconds must be between 0 and 65535")
    status = read_telemetry(bus, address)
    if not status["power_off_supported"]:
        raise ValueError("EC firmware does not support delayed power-off; flash the updated firmware")

    from smbus2 import i2c_msg

    payload = struct.pack("<BBH", CONTROL_REGISTER, POWER_OFF_COMMAND, delay_seconds)
    # A standalone write ends with STOP; no length byte, PEC, or repeated START.
    bus.i2c_rdwr(i2c_msg.write(address, payload))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=ADDRESS)
    parser.add_argument("--delay-seconds", type=int, required=True,
                        help="0..65535 seconds; 0 switches off after the write ends")
    args = parser.parse_args()
    if not 0x08 <= args.address <= 0x77:
        parser.error("address must be an unshifted 7-bit address from 0x08 to 0x77")
    if not 0 <= args.delay_seconds <= 65535:
        parser.error("delay-seconds must be between 0 and 65535")

    try:
        from smbus2 import SMBus

        print(f"Requesting load off in {args.delay_seconds} seconds.", flush=True)
        with SMBus(args.bus) as bus:
            request_power_off(bus, args.delay_seconds, args.address)
    except (ImportError, OSError, ValueError) as exc:
        parser.exit(1, f"EC power-off request failed: {exc}\n")
    print("Power-off command sent.", flush=True)


if __name__ == "__main__":
    main()
