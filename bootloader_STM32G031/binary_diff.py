#!/usr/bin/env python3
"""Binary file diff tool.

Compares two binary files of identical size and reports every differing byte.

Exit codes:
  0: files identical
  1: files differ
  2: error (including size mismatch)
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from typing import Generator, Iterable, Optional, Tuple


EXIT_SAME = 0
EXIT_DIFFERENT = 1
EXIT_ERROR = 2


@dataclass(frozen=True)
class DiffStats:
    size: int
    diff_count: int
    first_offset: Optional[int]
    last_offset: Optional[int]


def iter_differences(
    path_a: str,
    path_b: str,
    *,
    chunk_size: int,
) -> Generator[Tuple[int, int, int], None, None]:
    """Yield (offset, byte_a, byte_b) for each differing byte."""
    offset_base = 0
    with open(path_a, "rb") as fa, open(path_b, "rb") as fb:
        while True:
            ba = fa.read(chunk_size)
            bb = fb.read(chunk_size)

            if not ba and not bb:
                return

            # Size mismatch should have been checked already, but guard anyway.
            if len(ba) != len(bb):
                raise ValueError(
                    f"Files differ in size while reading (chunk mismatch at offset {offset_base})"
                )

            for i, (a, b) in enumerate(zip(ba, bb)):
                if a != b:
                    yield (offset_base + i, a, b)

            offset_base += len(ba)


def compute_stats(
    size: int,
    diffs: Iterable[Tuple[int, int, int]],
) -> DiffStats:
    diff_count = 0
    first_offset: Optional[int] = None
    last_offset: Optional[int] = None

    for off, _a, _b in diffs:
        diff_count += 1
        if first_offset is None:
            first_offset = off
        last_offset = off

    return DiffStats(
        size=size,
        diff_count=diff_count,
        first_offset=first_offset,
        last_offset=last_offset,
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Compare two same-size binary files byte-for-byte and list all differing offsets."
        )
    )
    p.add_argument("file1", help="First binary file")
    p.add_argument("file2", help="Second binary file")
    p.add_argument(
        "--chunk-size",
        type=int,
        default=1024 * 1024,
        help="Read size in bytes per chunk (default: 1048576)",
    )
    return p.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    try:
        size_a = os.path.getsize(args.file1)
        size_b = os.path.getsize(args.file2)
    except OSError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return EXIT_ERROR

    if size_a != size_b:
        print(
            f"ERROR: size mismatch: {args.file1} is {size_a} bytes, {args.file2} is {size_b} bytes",
            file=sys.stderr,
        )
        return EXIT_ERROR

    if args.chunk_size <= 0:
        print("ERROR: --chunk-size must be > 0", file=sys.stderr)
        return EXIT_ERROR

    # Pass 1: compute summary stats without buffering full diff output.
    try:
        stats = compute_stats(
            size=size_a,
            diffs=iter_differences(args.file1, args.file2, chunk_size=args.chunk_size),
        )
    except (OSError, ValueError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return EXIT_ERROR

    print(f"file1: {args.file1}")
    print(f"file2: {args.file2}")
    print(f"size_bytes: {stats.size}")
    print(f"diff_count: {stats.diff_count}")
    if stats.diff_count:
        assert stats.first_offset is not None and stats.last_offset is not None
        print(f"first_diff_offset_dec: {stats.first_offset}")
        print(f"first_diff_offset_hex: 0x{stats.first_offset:08X}")
        print(f"last_diff_offset_dec: {stats.last_offset}")
        print(f"last_diff_offset_hex: 0x{stats.last_offset:08X}")

    # Output diff lines.
    # Format: OFFSET_DEC\tOFFSET_HEX\tBYTE1_HEX\tBYTE2_HEX
    # Example: 16\t0x00000010\t0xAF\t0xB0
    if stats.diff_count:
        try:
            for off, a, b in iter_differences(
                args.file1, args.file2, chunk_size=args.chunk_size
            ):
                print(f"{off}\t0x{off:08X}\t0x{a:02X}\t0x{b:02X}")
        except (OSError, ValueError) as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return EXIT_ERROR

    return EXIT_SAME if stats.diff_count == 0 else EXIT_DIFFERENT


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

