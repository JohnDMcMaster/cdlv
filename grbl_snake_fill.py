#!/usr/bin/env python3
"""
GRBL laser: fill a rectangle with a horizontal snake raster (bidirectional passes).
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
DEFAULT_LPMM = 10.0
DEFAULT_FEED_MM_MIN = 1200.0


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Snake-fill a rectangle on a GRBL laser controller."
    )
    p.add_argument(
        "--width",
        type=float,
        required=True,
        metavar="MM",
        help="Rectangle width (mm).",
    )
    p.add_argument(
        "--height",
        type=float,
        required=True,
        metavar="MM",
        help="Rectangle height (mm).",
    )
    p.add_argument(
        "--lpmm",
        type=float,
        default=DEFAULT_LPMM,
        metavar="N",
        help="Scan line density in lines per mm (default: %(default)s).",
    )
    p.add_argument(
        "--start-x",
        type=float,
        default=0.0,
        metavar="MM",
        help="Rectangle origin X in machine coordinates (default: %(default)s).",
    )
    p.add_argument(
        "--start-y",
        type=float,
        default=0.0,
        metavar="MM",
        help="Rectangle origin Y in machine coordinates (default: %(default)s).",
    )
    p.add_argument(
        "--passes",
        type=int,
        default=1,
        metavar="N",
        help="Repeat the full fill this many times (ignored in dry-run; default: %(default)s).",
    )
    p.add_argument(
        "--fill",
        choices=("snake",),
        default="snake",
        help="Fill pattern (default: %(default)s).",
    )
    p.add_argument(
        "--power",
        type=int,
        default=100,
        metavar="S",
        help="Laser power for M3 S… (default: %(default)s, range 1–100).",
    )
    p.add_argument(
        "--no-dry",
        action="store_true",
        help=(
            "Run the real laser job. Default is dry-run: trace the rectangle perimeter "
            "once with the laser off (for locating your sample)."
        ),
    )
    p.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial device (default: {DEFAULT_PORT}).",
    )
    p.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate (default: {DEFAULT_BAUD}).",
    )
    p.add_argument(
        "--feed",
        type=float,
        default=DEFAULT_FEED_MM_MIN,
        metavar="MM/MIN",
        help=f"Feed rate for G1 moves (default: {DEFAULT_FEED_MM_MIN}).",
    )
    args = p.parse_args(argv)
    if args.width <= 0 or args.height <= 0:
        p.error("width and height must be positive.")
    if args.lpmm <= 0:
        p.error("--lpmm must be positive.")
    if args.passes < 1:
        p.error("--passes must be at least 1.")
    if not 1 <= args.power <= 100:
        p.error("--power must be between 1 and 100.")
    return args


def row_count(height_mm: float, lpmm: float) -> int:
    n = round(height_mm * lpmm)
    return max(1, n)


def row_y_positions(height_mm: float, rows: int) -> list[float]:
    if rows == 1:
        return [0.0]
    return [height_mm * i / (rows - 1) for i in range(rows)]


def read_grbl_responses(ser: serial.Serial, timeout_s: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = ser.readline()
        if not line:
            continue
        decoded = line.decode(errors="replace").strip()
        if not decoded:
            continue
        lower = decoded.lower()
        if "error" in lower or lower.startswith("alarm"):
            raise RuntimeError(f"GRBL: {decoded}")
        if lower == "ok":
            return
    raise TimeoutError("Timed out waiting for GRBL 'ok'.")


def send_line(ser: serial.Serial, line: str) -> None:
    ser.write((line.strip() + "\r\n").encode("ascii", errors="strict"))
    read_grbl_responses(ser)


def ensure_laser_off(ser: serial.Serial) -> None:
    try:
        send_line(ser, "M5")
    except Exception:
        pass


def run_dry_perimeter(
    ser: serial.Serial,
    sx: float,
    sy: float,
    width: float,
    height: float,
    feed: float,
) -> None:
    send_line(ser, "G21")
    send_line(ser, "G90")
    send_line(ser, "M5")
    x0, y0 = sx, sy
    x1, y1 = sx + width, sy
    x2, y2 = sx + width, sy + height
    x3, y3 = sx, sy + height
    send_line(ser, f"G0 X{x0:.4f} Y{y0:.4f}")
    send_line(ser, f"G1 X{x1:.4f} Y{y1:.4f} F{feed:.2f}")
    send_line(ser, f"G1 X{x2:.4f} Y{y2:.4f} F{feed:.2f}")
    send_line(ser, f"G1 X{x3:.4f} Y{y3:.4f} F{feed:.2f}")
    send_line(ser, f"G1 X{x0:.4f} Y{y0:.4f} F{feed:.2f}")
    send_line(ser, f"G0 X{x0:.4f} Y{y0:.4f}")


def run_snake_fill(
    ser: serial.Serial,
    sx: float,
    sy: float,
    width: float,
    ys: list[float],
    passes: int,
    power: int,
    feed: float,
) -> None:
    send_line(ser, "G21")
    send_line(ser, "G90")
    send_line(ser, f"G0 X{sx:.4f} Y{sy:.4f}")

    send_line(ser, f"M3 S{power}")

    for p in range(passes):
        for i, y_local in enumerate(ys):
            y = sy + y_local
            if i % 2 == 0:
                x0, x1 = sx, sx + width
            else:
                x0, x1 = sx + width, sx

            first_segment = p == 0 and i == 0
            if not first_segment:
                send_line(ser, "M5")
                send_line(ser, f"G0 X{x0:.4f} Y{y:.4f}")
                send_line(ser, f"M3 S{power}")
            send_line(ser, f"G1 X{x1:.4f} Y{y:.4f} F{feed:.2f}")

    send_line(ser, "M5")
    send_line(ser, f"G0 X{sx:.4f} Y{sy:.4f}")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    dry_run = not args.no_dry
    rows = row_count(args.height, args.lpmm)
    ys = row_y_positions(args.height, rows)
    sx, sy = args.start_x, args.start_y

    ser: serial.Serial | None = None
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=2.0,
            write_timeout=2.0,
        )
        time.sleep(2.0)
        ser.reset_input_buffer()

        if dry_run:
            run_dry_perimeter(ser, sx, sy, args.width, args.height, args.feed)
        else:
            run_snake_fill(
                ser,
                sx,
                sy,
                args.width,
                ys,
                args.passes,
                args.power,
                args.feed,
            )
    finally:
        if ser is not None:
            ensure_laser_off(ser)
            ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
