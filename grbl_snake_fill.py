#!/usr/bin/env python3
"""
GRBL laser: fill a rectangle with a horizontal snake raster (bidirectional passes).
"""

from __future__ import annotations

import argparse
import itertools
import re
import sys
import time

import serial

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
DEFAULT_LPMM = 10.0
DEFAULT_FEED_MM_MIN = 1200.0

# Grbl 1.1 gcode.c — common execution errors (subset).
GRBL_ERROR_HELP: dict[int, str] = {
    1: "Expected command letter (bad line, blank line, or serial corruption).",
    2: "Bad number format.",
    3: "Invalid statement (unsupported command).",
    4: "Value < 0.",
    5: "Modal group violation.",
    6: "Undefined feed rate.",
    7: "Command requires integer.",
    8: "Axis command conflict.",
    9: "Word repeated in block.",
    10: "Command requires a value.",
    11: "G53 requires G0 or G1.",
    12: "Axis words missing.",
    13: "Invalid line number.",
    14: "Value out of range.",
    15: "Too many words in block.",
    16: "Homing not enabled.",
    17: "Line overflow.",
    18: "RPM out of range.",
    20: "Unsupported command.",
    21: "Modal group violation (motion).",
    22: "Invalid value for word.",
    23: "G-code sentence not supported.",
    24: "Axis words missing in plane selection.",
    25: "Invalid arc plane.",
}


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
        help=(
            "Repeat the full fill this many times; use 0 to run until interrupted "
            "(ignored in dry-run; default: %(default)s)."
        ),
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
    p.add_argument(
        "--verbose",
        action="store_true",
        help="Log each sent line and GRBL responses to stderr.",
    )
    p.add_argument(
        "--line-delay",
        type=float,
        default=0.0,
        metavar="SEC",
        help="Optional pause after each acknowledged line (helps flaky USB; default: 0).",
    )
    args = p.parse_args(argv)
    if args.width <= 0 or args.height <= 0:
        p.error("width and height must be positive.")
    if args.lpmm <= 0:
        p.error("--lpmm must be positive.")
    if args.passes < 0:
        p.error("--passes must be non-negative (0 = run forever).")
    if not 1 <= args.power <= 100:
        p.error("--power must be between 1 and 100.")
    if args.line_delay < 0:
        p.error("--line-delay must be non-negative.")
    return args


def row_count(height_mm: float, lpmm: float) -> int:
    n = round(height_mm * lpmm)
    return max(1, n)


def row_y_positions(height_mm: float, rows: int) -> list[float]:
    if rows == 1:
        return [0.0]
    return [height_mm * i / (rows - 1) for i in range(rows)]


def fmt_axis(v: float) -> str:
    """Format a coordinate for GRBL (3 decimals, trim trailing zeros)."""
    s = f"{v:.3f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s if s else "0"


def fmt_feed(f: float) -> str:
    s = f"{f:.2f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s if s else "0"


def _line_requires_motion_sync(line: str) -> bool:
    """True for G0/G00/G1/G01 blocks (motion that should finish before the next command)."""
    s = line.strip().upper()
    if not s.startswith("G"):
        return False
    i = 1
    while i < len(s) and s[i].isdigit():
        i += 1
    gword = s[:i]
    return gword in ("G0", "G00", "G1", "G01")


_RE_ERROR = re.compile(r"^error:\s*(\d+)", re.I)


def _classify_line(decoded: str) -> str:
    if not decoded:
        return "empty"
    if decoded.lower() == "ok":
        return "ok"
    if _RE_ERROR.match(decoded):
        return "error"
    if decoded.startswith("<") and "|" in decoded:
        return "status"
    if decoded.startswith("[") and decoded.endswith("]"):
        return "bracket"
    if decoded.lower().startswith("grbl"):
        return "welcome"
    if "error" in decoded.lower() or decoded.lower().startswith("alarm"):
        return "errorish"
    return "other"


def read_until_ok(
    ser: serial.Serial,
    timeout_s: float = 30.0,
    verbose: bool = False,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = ser.readline()
        if not line:
            continue
        decoded = line.decode(errors="replace").strip()
        if not decoded:
            continue
        if verbose:
            print(f"< {decoded}", file=sys.stderr)
        kind = _classify_line(decoded)
        if kind == "ok":
            return
        if kind == "error":
            m = _RE_ERROR.match(decoded)
            code = int(m.group(1)) if m else -1
            hint = GRBL_ERROR_HELP.get(code, "")
            extra = f" {hint}" if hint else ""
            raise RuntimeError(f"GRBL: {decoded}{extra}")
        if kind == "errorish":
            raise RuntimeError(f"GRBL: {decoded}")
        # status, bracket, welcome, other: keep reading until ok
    raise TimeoutError("Timed out waiting for GRBL 'ok'.")


def wait_for_idle(
    ser: serial.Serial,
    *,
    verbose: bool = False,
    timeout_s: float = 600.0,
) -> None:
    """
    Poll GRBL with realtime '?' until status reports Idle.

    A line's 'ok' only means the block was accepted into the planner, not that
    motion finished; this waits for the machine to finish the current moves.
    """
    deadline = time.monotonic() + timeout_s
    saved_timeout = ser.timeout
    ser.timeout = 0.2
    try:
        while time.monotonic() < deadline:
            ser.write(b"?")
            inner = time.monotonic() + 0.4
            while time.monotonic() < inner:
                raw = ser.readline()
                if not raw:
                    break
                text = raw.decode(errors="replace").strip()
                if verbose:
                    print(f"< ? {text}", file=sys.stderr)
                if text.startswith("<") and "|" in text:
                    state = text[1:].split("|")[0]
                    if state == "Idle":
                        return
                    if state.startswith("Alarm") or state == "Check":
                        raise RuntimeError(f"GRBL cannot reach Idle: {text}")
        raise TimeoutError("Timed out waiting for GRBL Idle.")
    finally:
        ser.timeout = saved_timeout


def drain_serial(ser: serial.Serial, verbose: bool = False) -> None:
    """Discard boot banner and stray bytes (time-bounded; cannot hang on status spam)."""
    t_end = time.monotonic() + 1.5
    while time.monotonic() < t_end:
        n = ser.in_waiting
        if n:
            ser.read(n)
            if verbose:
                print(f"< [drain {n}b]", file=sys.stderr)
        else:
            time.sleep(0.02)


def send_line(
    ser: serial.Serial,
    line: str,
    *,
    verbose: bool = False,
    line_delay: float = 0.0,
    retries: int = 3,
) -> None:
    line = line.strip()
    if not line:
        raise ValueError("Refusing to send an empty G-code line.")
    last_err: RuntimeError | None = None
    for attempt in range(retries):
        if attempt and verbose:
            print(f"(retry {attempt + 1}/{retries})", file=sys.stderr)
        if verbose:
            print(f"> {line}", file=sys.stderr)
        ser.write((line + "\r\n").encode("ascii", errors="strict"))
        ser.flush()
        try:
            read_until_ok(ser, verbose=verbose)
            if line_delay > 0:
                time.sleep(line_delay)
            if _line_requires_motion_sync(line):
                wait_for_idle(ser, verbose=verbose)
            return
        except RuntimeError as e:
            last_err = e
            msg = str(e).lower()
            if "error:1" not in msg and "expected command letter" not in msg:
                raise
            if attempt >= retries - 1:
                raise
            time.sleep(0.03 * (2**attempt))
            drain_serial(ser, verbose=verbose)
    assert last_err is not None
    raise last_err


def ensure_laser_off(
    ser: serial.Serial,
    verbose: bool = False,
    line_delay: float = 0.0,
) -> None:
    try:
        send_line(ser, "M5", verbose=verbose, line_delay=line_delay)
    except Exception:
        pass


def run_dry_perimeter(
    ser: serial.Serial,
    sx: float,
    sy: float,
    width: float,
    height: float,
    feed: float,
    *,
    verbose: bool,
    line_delay: float,
) -> None:
    send_line(ser, "G21", verbose=verbose, line_delay=line_delay)
    send_line(ser, "G90", verbose=verbose, line_delay=line_delay)
    send_line(ser, "M5", verbose=verbose, line_delay=line_delay)
    x0, y0 = sx, sy
    x1, y1 = sx + width, sy
    x2, y2 = sx + width, sy + height
    x3, y3 = sx, sy + height
    fx, fy = fmt_axis, fmt_axis
    ff = fmt_feed
    send_line(
        ser,
        f"G0 X{fx(x0)} Y{fy(y0)}",
        verbose=verbose,
        line_delay=line_delay,
    )
    send_line(
        ser,
        f"G1 X{fx(x1)} Y{fy(y1)} F{ff(feed)}",
        verbose=verbose,
        line_delay=line_delay,
    )
    send_line(
        ser,
        f"G1 X{fx(x2)} Y{fy(y2)} F{ff(feed)}",
        verbose=verbose,
        line_delay=line_delay,
    )
    send_line(
        ser,
        f"G1 X{fx(x3)} Y{fy(y3)} F{ff(feed)}",
        verbose=verbose,
        line_delay=line_delay,
    )
    send_line(
        ser,
        f"G1 X{fx(x0)} Y{fy(y0)} F{ff(feed)}",
        verbose=verbose,
        line_delay=line_delay,
    )
    send_line(
        ser,
        f"G0 X{fx(x0)} Y{fy(y0)}",
        verbose=verbose,
        line_delay=line_delay,
    )


def run_snake_fill(
    ser: serial.Serial,
    sx: float,
    sy: float,
    width: float,
    ys: list[float],
    passes: int,
    power: int,
    feed: float,
    *,
    verbose: bool,
    line_delay: float,
) -> None:
    fx, fy = fmt_axis, fmt_axis
    ff = fmt_feed
    send_line(ser, "G21", verbose=verbose, line_delay=line_delay)
    send_line(ser, "G90", verbose=verbose, line_delay=line_delay)
    send_line(
        ser,
        f"G0 X{fx(sx)} Y{fy(sy)}",
        verbose=verbose,
        line_delay=line_delay,
    )

    send_line(ser, f"M3 S{power}", verbose=verbose, line_delay=line_delay)

    pass_iter = itertools.count(0) if passes == 0 else range(passes)
    for p in pass_iter:
        for i, y_local in enumerate(ys):
            y = sy + y_local
            if i % 2 == 0:
                x0, x1 = sx, sx + width
            else:
                x0, x1 = sx + width, sx

            first_segment = p == 0 and i == 0
            if not first_segment:
                send_line(ser, "M5", verbose=verbose, line_delay=line_delay)
                send_line(
                    ser,
                    f"G0 X{fx(x0)} Y{fy(y)}",
                    verbose=verbose,
                    line_delay=line_delay,
                )
                send_line(
                    ser,
                    f"M3 S{power}",
                    verbose=verbose,
                    line_delay=line_delay,
                )
            send_line(
                ser,
                f"G1 X{fx(x1)} Y{fy(y)} F{ff(feed)}",
                verbose=verbose,
                line_delay=line_delay,
            )

    send_line(ser, "M5", verbose=verbose, line_delay=line_delay)
    send_line(
        ser,
        f"G0 X{fx(sx)} Y{fy(sy)}",
        verbose=verbose,
        line_delay=line_delay,
    )


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    dry_run = not args.no_dry
    rows = row_count(args.height, args.lpmm)
    ys = row_y_positions(args.height, rows)
    sx, sy = args.start_x, args.start_y
    verbose = args.verbose
    line_delay = args.line_delay

    ser: serial.Serial | None = None
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=2.0,
            write_timeout=2.0,
        )
        time.sleep(2.0)
        drain_serial(ser, verbose=verbose)

        if dry_run:
            run_dry_perimeter(
                ser,
                sx,
                sy,
                args.width,
                args.height,
                args.feed,
                verbose=verbose,
                line_delay=line_delay,
            )
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
                verbose=verbose,
                line_delay=line_delay,
            )
    finally:
        if ser is not None:
            ensure_laser_off(ser, verbose=verbose, line_delay=line_delay)
            ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
