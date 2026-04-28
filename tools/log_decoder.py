#!/usr/bin/env python3
"""ACS4 binary flight log decoder.

Reads LOG_NNN.BIN files produced by the ACS4 flight logger and converts
them to CSV or prints a human-readable summary.

The binary format mirrors the packed C structs defined in
``src/logger/log_format.h``.  All multi-byte values are little-endian
(ARM Cortex-M7 native).

Usage::

    # Decode to CSV (one file per message type)
    python tools/log_decoder.py LOG_001.BIN -o output_dir/

    # Print summary to stdout
    python tools/log_decoder.py LOG_001.BIN --summary

    # Dump all records to stdout
    python tools/log_decoder.py LOG_001.BIN --dump
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO

# ---------------------------------------------------------------------------
# Wire format constants (must match log_format.h)
# ---------------------------------------------------------------------------

FILE_MAGIC = b"ACS4"
FORMAT_VERSION = 1

HEADER_SIZE = 5  # uint8 msg_id + uint32 timestamp_us
FILE_HEADER_SIZE = 14  # magic(4) + version(2) + sysclk(4) + boot_ms(4)

# Message IDs
MSG_IMU = 0x01
MSG_NAV = 0x02
MSG_CTRL = 0x03
MSG_BARO = 0x04
MSG_MAG = 0x05
MSG_EVENT = 0x06

# struct formats (little-endian)
FMT_HEADER = "<BI"  # msg_id(u8), timestamp_us(u32)
FMT_FILE_HEADER = "<4sHII"  # magic(4s), version(u16), sysclk(u32), boot_ms(u32)
FMT_IMU = "<BI3h3h"  # header + accel[3] + gyro[3]
FMT_NAV = "<BI4h3i3h"  # header + quat[4] + pos[3] + vel[3]
FMT_CTRL = "<BI4hB"  # header + servo[4] + flight_state
FMT_BARO = "<BIIi"  # header + pressure_pa(u32) + altitude_mm(i32)
FMT_MAG = "<BI3h"  # header + field[3]
FMT_EVENT = "<BIBH"  # header + event_code(u8) + aux(u16)

MSG_SIZES: dict[int, int] = {
    MSG_IMU: struct.calcsize(FMT_IMU),
    MSG_NAV: struct.calcsize(FMT_NAV),
    MSG_CTRL: struct.calcsize(FMT_CTRL),
    MSG_BARO: struct.calcsize(FMT_BARO),
    MSG_MAG: struct.calcsize(FMT_MAG),
    MSG_EVENT: struct.calcsize(FMT_EVENT),
}

MSG_NAMES: dict[int, str] = {
    MSG_IMU: "IMU",
    MSG_NAV: "NAV",
    MSG_CTRL: "CTRL",
    MSG_BARO: "BARO",
    MSG_MAG: "MAG",
    MSG_EVENT: "EVENT",
}


# ---------------------------------------------------------------------------
# Decoded record types
# ---------------------------------------------------------------------------


@dataclass
class FileHeader:
    magic: bytes
    version: int
    sysclk_hz: int
    boot_time_ms: int


@dataclass
class ImuRecord:
    timestamp_us: int
    accel_mps2: tuple[float, float, float]
    gyro_rads: tuple[float, float, float]


@dataclass
class NavRecord:
    timestamp_us: int
    quat: tuple[float, float, float, float]
    pos_m: tuple[float, float, float]
    vel_mps: tuple[float, float, float]


@dataclass
class CtrlRecord:
    timestamp_us: int
    servo_deg: tuple[float, float, float, float]
    flight_state: int


@dataclass
class BaroRecord:
    timestamp_us: int
    pressure_pa: int
    altitude_m: float


@dataclass
class MagRecord:
    timestamp_us: int
    field_ut: tuple[float, float, float]


@dataclass
class EventRecord:
    timestamp_us: int
    event_code: int
    aux: int


@dataclass
class DecodedLog:
    header: FileHeader | None = None
    imu: list[ImuRecord] = field(default_factory=list)
    nav: list[NavRecord] = field(default_factory=list)
    ctrl: list[CtrlRecord] = field(default_factory=list)
    baro: list[BaroRecord] = field(default_factory=list)
    mag: list[MagRecord] = field(default_factory=list)
    events: list[EventRecord] = field(default_factory=list)
    unknown_count: int = 0
    parse_errors: int = 0


# ---------------------------------------------------------------------------
# Decoder
# ---------------------------------------------------------------------------


def decode_file_header(data: bytes) -> FileHeader:
    magic, version, sysclk, boot_ms = struct.unpack(FMT_FILE_HEADER, data)
    return FileHeader(
        magic=magic,
        version=version,
        sysclk_hz=sysclk,
        boot_time_ms=boot_ms,
    )


def decode_log(fp: BinaryIO) -> DecodedLog:
    """Decode an entire binary log file into structured records."""
    log = DecodedLog()

    raw_hdr = fp.read(FILE_HEADER_SIZE)
    if len(raw_hdr) < FILE_HEADER_SIZE:
        print("Error: file too short for header", file=sys.stderr)
        return log

    log.header = decode_file_header(raw_hdr)

    if log.header.magic != FILE_MAGIC:
        print(
            f"Error: bad magic {log.header.magic!r} (expected {FILE_MAGIC!r})",
            file=sys.stderr,
        )
        return log

    if log.header.version != FORMAT_VERSION:
        print(
            f"Warning: format version {log.header.version} "
            f"(decoder expects {FORMAT_VERSION})",
            file=sys.stderr,
        )

    while True:
        msg_id_byte = fp.read(1)
        if len(msg_id_byte) == 0:
            break

        msg_id = msg_id_byte[0]

        rec_size = MSG_SIZES.get(msg_id)
        if rec_size is None:
            log.unknown_count += 1
            continue

        remaining = rec_size - 1
        payload = fp.read(remaining)
        if len(payload) < remaining:
            log.parse_errors += 1
            break

        raw = msg_id_byte + payload

        try:
            _decode_record(log, msg_id, raw)
        except struct.error:
            log.parse_errors += 1

    return log


def _decode_record(log: DecodedLog, msg_id: int, raw: bytes) -> None:
    if msg_id == MSG_IMU:
        _, ts, ax, ay, az, gx, gy, gz = struct.unpack(FMT_IMU, raw)
        log.imu.append(
            ImuRecord(
                timestamp_us=ts,
                accel_mps2=(ax * 0.001, ay * 0.001, az * 0.001),
                gyro_rads=(gx * 0.01, gy * 0.01, gz * 0.01),
            )
        )

    elif msg_id == MSG_NAV:
        _, ts, qw, qx, qy, qz, px, py, pz, vx, vy, vz = struct.unpack(FMT_NAV, raw)
        log.nav.append(
            NavRecord(
                timestamp_us=ts,
                quat=(qw / 32767.0, qx / 32767.0, qy / 32767.0, qz / 32767.0),
                pos_m=(px * 0.001, py * 0.001, pz * 0.001),
                vel_mps=(vx * 0.01, vy * 0.01, vz * 0.01),
            )
        )

    elif msg_id == MSG_CTRL:
        _, ts, s0, s1, s2, s3, fs = struct.unpack(FMT_CTRL, raw)
        log.ctrl.append(
            CtrlRecord(
                timestamp_us=ts,
                servo_deg=(s0 * 0.01, s1 * 0.01, s2 * 0.01, s3 * 0.01),
                flight_state=fs,
            )
        )

    elif msg_id == MSG_BARO:
        _, ts, press, alt_mm = struct.unpack(FMT_BARO, raw)
        log.baro.append(
            BaroRecord(
                timestamp_us=ts,
                pressure_pa=press,
                altitude_m=alt_mm * 0.001,
            )
        )

    elif msg_id == MSG_MAG:
        _, ts, mx, my, mz = struct.unpack(FMT_MAG, raw)
        log.mag.append(
            MagRecord(
                timestamp_us=ts,
                field_ut=(mx * 0.01, my * 0.01, mz * 0.01),
            )
        )

    elif msg_id == MSG_EVENT:
        _, ts, code, aux = struct.unpack(FMT_EVENT, raw)
        log.events.append(EventRecord(timestamp_us=ts, event_code=code, aux=aux))


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------


def print_summary(log: DecodedLog) -> None:
    """Print a human-readable summary of the decoded log."""
    hdr = log.header
    if hdr is None:
        print("No valid file header found.")
        return

    print("=== ACS4 Flight Log ===")
    print(f"Format version: {hdr.version}")
    print(f"System clock:   {hdr.sysclk_hz / 1e6:.1f} MHz")
    print(f"Boot time:      {hdr.boot_time_ms} ms")
    print()

    total = (
        len(log.imu)
        + len(log.nav)
        + len(log.ctrl)
        + len(log.baro)
        + len(log.mag)
        + len(log.events)
    )
    print(f"Records decoded: {total}")
    print(f"  IMU:    {len(log.imu)}")
    print(f"  NAV:    {len(log.nav)}")
    print(f"  CTRL:   {len(log.ctrl)}")
    print(f"  BARO:   {len(log.baro)}")
    print(f"  MAG:    {len(log.mag)}")
    print(f"  EVENT:  {len(log.events)}")
    print(f"  Unknown: {log.unknown_count}")
    print(f"  Errors:  {log.parse_errors}")

    if log.imu:
        t0 = log.imu[0].timestamp_us
        t1 = log.imu[-1].timestamp_us
        dt = (t1 - t0) / 1e6
        rate = len(log.imu) / dt if dt > 0 else 0
        print(f"\nIMU: {dt:.2f}s duration, ~{rate:.0f} Hz")

    if log.baro:
        t0 = log.baro[0].timestamp_us
        t1 = log.baro[-1].timestamp_us
        dt = (t1 - t0) / 1e6
        rate = len(log.baro) / dt if dt > 0 else 0
        print(f"BARO: {dt:.2f}s duration, ~{rate:.0f} Hz")


def dump_records(log: DecodedLog) -> None:
    """Print all records to stdout in chronological order."""
    lines: list[tuple[int, str]] = []

    for r in log.imu:
        ax, ay, az = r.accel_mps2
        gx, gy, gz = r.gyro_rads
        lines.append(
            (
                r.timestamp_us,
                f"IMU  t={r.timestamp_us:>10} "
                f"a=[{ax:+8.3f} {ay:+8.3f} {az:+8.3f}] "
                f"g=[{gx:+7.4f} {gy:+7.4f} {gz:+7.4f}]",
            )
        )

    for rb in log.baro:
        lines.append(
            (
                rb.timestamp_us,
                f"BARO t={rb.timestamp_us:>10} "
                f"P={rb.pressure_pa} Pa  alt={rb.altitude_m:.2f} m",
            )
        )

    for rm in log.mag:
        mx, my, mz = rm.field_ut
        lines.append(
            (
                rm.timestamp_us,
                f"MAG  t={rm.timestamp_us:>10} B=[{mx:+7.2f} {my:+7.2f} {mz:+7.2f}] uT",
            )
        )

    for rn in log.nav:
        qw, qx, qy, qz = rn.quat
        px, py, pz = rn.pos_m
        vx, vy, vz = rn.vel_mps
        lines.append(
            (
                rn.timestamp_us,
                f"NAV  t={rn.timestamp_us:>10} "
                f"q=[{qw:+.4f} {qx:+.4f} {qy:+.4f} {qz:+.4f}] "
                f"p=[{px:+.2f} {py:+.2f} {pz:+.2f}] "
                f"v=[{vx:+.2f} {vy:+.2f} {vz:+.2f}]",
            )
        )

    for rc in log.ctrl:
        s0, s1, s2, s3 = rc.servo_deg
        lines.append(
            (
                rc.timestamp_us,
                f"CTRL t={rc.timestamp_us:>10} "
                f"srv=[{s0:+6.2f} {s1:+6.2f} {s2:+6.2f} {s3:+6.2f}] "
                f"state={rc.flight_state}",
            )
        )

    for re_ in log.events:
        lines.append(
            (
                re_.timestamp_us,
                f"EVT  t={re_.timestamp_us:>10} code={re_.event_code} aux={re_.aux}",
            )
        )

    lines.sort(key=lambda x: x[0])
    for _, line in lines:
        print(line)


def export_csv(log: DecodedLog, output_dir: Path) -> None:
    """Export decoded log to CSV files (one per message type)."""
    output_dir.mkdir(parents=True, exist_ok=True)

    if log.imu:
        path = output_dir / "imu.csv"
        with path.open("w") as f:
            f.write("timestamp_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n")
            for r in log.imu:
                ax, ay, az = r.accel_mps2
                gx, gy, gz = r.gyro_rads
                f.write(
                    f"{r.timestamp_us},{ax:.4f},{ay:.4f},{az:.4f},"
                    f"{gx:.5f},{gy:.5f},{gz:.5f}\n"
                )
        print(f"  Wrote {path} ({len(log.imu)} records)")

    if log.nav:
        path = output_dir / "nav.csv"
        with path.open("w") as f:
            f.write("timestamp_us,qw,qx,qy,qz,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d\n")
            for rn in log.nav:
                qw, qx, qy, qz = rn.quat
                pn, pe, pd = rn.pos_m
                vn, ve, vd = rn.vel_mps
                f.write(
                    f"{rn.timestamp_us},{qw:.6f},{qx:.6f},{qy:.6f},{qz:.6f},"
                    f"{pn:.4f},{pe:.4f},{pd:.4f},"
                    f"{vn:.4f},{ve:.4f},{vd:.4f}\n"
                )
        print(f"  Wrote {path} ({len(log.nav)} records)")

    if log.ctrl:
        path = output_dir / "ctrl.csv"
        with path.open("w") as f:
            f.write("timestamp_us,servo0,servo1,servo2,servo3,flight_state\n")
            for rc in log.ctrl:
                s0, s1, s2, s3 = rc.servo_deg
                f.write(
                    f"{rc.timestamp_us},{s0:.2f},{s1:.2f},{s2:.2f},{s3:.2f},"
                    f"{rc.flight_state}\n"
                )
        print(f"  Wrote {path} ({len(log.ctrl)} records)")

    if log.baro:
        path = output_dir / "baro.csv"
        with path.open("w") as f:
            f.write("timestamp_us,pressure_pa,altitude_m\n")
            for rb in log.baro:
                f.write(f"{rb.timestamp_us},{rb.pressure_pa},{rb.altitude_m:.3f}\n")
        print(f"  Wrote {path} ({len(log.baro)} records)")

    if log.mag:
        path = output_dir / "mag.csv"
        with path.open("w") as f:
            f.write("timestamp_us,field_x,field_y,field_z\n")
            for rm in log.mag:
                mx, my, mz = rm.field_ut
                f.write(f"{rm.timestamp_us},{mx:.3f},{my:.3f},{mz:.3f}\n")
        print(f"  Wrote {path} ({len(log.mag)} records)")

    if log.events:
        path = output_dir / "events.csv"
        with path.open("w") as f:
            f.write("timestamp_us,event_code,aux\n")
            for re_ in log.events:
                f.write(f"{re_.timestamp_us},{re_.event_code},{re_.aux}\n")
        print(f"  Wrote {path} ({len(log.events)} records)")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Decode ACS4 binary flight logs to CSV or human-readable text."
    )
    parser.add_argument("logfile", type=Path, help="Path to LOG_NNN.BIN file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output directory for CSV files",
    )
    parser.add_argument(
        "--summary", action="store_true", help="Print summary to stdout"
    )
    parser.add_argument(
        "--dump", action="store_true", help="Dump all records to stdout"
    )

    args = parser.parse_args()

    if not args.logfile.exists():
        print(f"Error: file not found: {args.logfile}", file=sys.stderr)
        return 1

    with args.logfile.open("rb") as fp:
        log = decode_log(fp)

    if args.summary or (not args.output and not args.dump):
        print_summary(log)

    if args.dump:
        dump_records(log)

    if args.output:
        print(f"Exporting to {args.output}/")
        export_csv(log, args.output)

    return 0


if __name__ == "__main__":
    sys.exit(main())
