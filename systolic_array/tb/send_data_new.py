#!/usr/bin/env python3
"""
send_data.py - talks to Top_FPGA over the board's USB-UART.

Generalized for a ROWS x COLS systolic array (weight matrix ROWS x COLS,
square ROWS x ROWS activation matrix, ROWS x COLS output).

Sends ROWS*COLS + ROWS*ROWS signed 24-bit words (weights row-major, then
ROWS activation rows of ROWS entries each), each as 3 bytes MSB-first,
then reads back ROWS*COLS signed 24-bit results (+ a 4-byte cycle count)
and prints the result as a ROWS x COLS matrix.

Install once:  pip install pyserial
Run:           python3 send_data_new.py
"""

import serial
import time

# ---- EDIT THESE ----
PORT = "COM14"   # Linux: /dev/ttyUSBx (often the *second* CDC port
                         # Vivado's programmer uses the first). Windows: "COM5"
                         # style - check Device Manager. Run `dmesg | tail`
                         # right after plugging in on Linux to see which one.
BAUD = 115200
WIDTH_BYTES = 3          # 24-bit words -> 3 bytes each

# ---- array shape - MUST match ROWS/COLS synthesized into Top_FPGA ----
ROWS = 3   # weight matrix rows / activation matrix side (square)
COLS = 3   # weight matrix columns / output width

# ---- your data ----
# ROWS*COLS weights, row-major: w[0][0] w[0][1] ... w[0][COLS-1] w[1][0] ...
weights = [
    1, 2, 3,
    4, 5, 6,
    7, 8, 9
]

# ROWS activation rows, ROWS entries each: row0, row1, row2, row3
rows = [
    [2, 1, 0],   # row0
    [1, 2, 1],   # row1
    [0, 1, 2]    # row2
]



def to_bytes_signed24(value: int) -> bytes:
    """Pack a signed int into 3 bytes, MSB first, two's complement."""
    if value < 0:
        value = value + (1 << 24)
    if not (0 <= value < (1 << 24)):
        raise ValueError(f"{value} does not fit in 24 bits")
    return value.to_bytes(3, byteorder="big", signed=False)


def from_bytes_signed24(b: bytes) -> int:
    """Unpack 3 MSB-first bytes into a signed 24-bit int."""
    value = int.from_bytes(b, byteorder="big", signed=False)
    if value & (1 << 23):        # sign bit set -> negative
        value -= (1 << 24)
    return value


def main():
    assert len(weights) == ROWS * COLS, f"expected {ROWS*COLS} weights, got {len(weights)}"
    assert len(rows) == ROWS and all(len(r) == ROWS for r in rows), \
        f"expected a {ROWS}x{ROWS} activation matrix"

    words = list(weights)
    for r in rows:
        words.extend(r)

    total_words = ROWS * COLS + ROWS * ROWS
    assert len(words) == total_words

    payload = b"".join(to_bytes_signed24(w) for w in words)
    assert len(payload) == total_words * WIDTH_BYTES

    result_words = ROWS * COLS
    result_bytes_expected = result_words * WIDTH_BYTES + 4  # + 4-byte cycle count

    with serial.Serial(PORT, BAUD, timeout=5) as ser:
        print(f"Opened {PORT} @ {BAUD} baud")

        print(f"Sending {len(payload)} bytes "
              f"({ROWS*COLS} weights + {ROWS*ROWS} activations)...")
        ser.write(payload)
        ser.flush()

        print(f"Waiting for {result_bytes_expected} result bytes...")
        result = ser.read(result_bytes_expected)

        if len(result) < result_bytes_expected:
            print(f"Only got {len(result)} of {result_bytes_expected} bytes back - "
                  f"check wiring, baud rate, or that 'done' actually pulsed on the board.")
            return

        values = [
            from_bytes_signed24(result[i:i + 3])
            for i in range(0, result_words * WIDTH_BYTES, 3)
        ]
        cycle_count = int.from_bytes(result[result_words * WIDTH_BYTES:], byteorder="big", signed=False)

        # values arrive as: addr0(col0..col{COLS-1}), addr1(...), ..., addr{ROWS-1}(...)
        print(f"\nResult matrix ({ROWS}x{COLS}, rows = addr 0..{ROWS-1}, "
              f"cols = col0..col{COLS-1}):")
        for addr in range(ROWS):
            row_vals = values[addr * COLS:(addr + 1) * COLS]
            print("  " + "  ".join(f"{v:>8}" for v in row_vals))

        print(f"\nCycle count (start -> done): {cycle_count}")


if __name__ == "__main__":
    main()
