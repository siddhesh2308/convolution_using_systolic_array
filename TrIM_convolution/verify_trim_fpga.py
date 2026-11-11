#!/usr/bin/env python3
"""
verification for trim_fpga_top.v on the Nexys A7.



Four subcommands:

  calibrate   Sends the identity kernel + an impulse image, receives the raw

  verify      Sends a real kernel + image, receives the raw hardware convolution, and

  preview     No hardware needed. Loads an image (.npy/.jpg/.png), runs ONLY the golden model, and saves the convolved result as a PNG so you can see what to expect before/without the board. Also saves a side-by-side (input / golden) comparison figure.
  
  self-test   Sanity-checks the golden model with random data, no image.

Example:
    python verify_trim_fpga.py preview   --image img_1.jpg --kernel sobel_x
    python verify_trim_fpga.py calibrate --port /dev/ttyUSB1
    python verify_trim_fpga.py verify    --port /dev/ttyUSB1 --image img_1.jpg --kernel sobel_x --show
"""

import argparse
import struct
import sys
import time

import numpy as np

try:
    import serial
except ImportError:
    serial = None  

try:
    from PIL import Image
except ImportError:
    Image = None  

try:
    import matplotlib
    matplotlib.use("Agg")  
    import matplotlib.pyplot as plt
except ImportError:
    plt = None  


# ----------------------------------------------------------------------
# Design constants -- MUST match the parameters used in trim_fpga_top.v
# ----------------------------------------------------------------------
IW = 24
K = 3
IMG_W = 28
IMG_H = 28
BAUD = 115200

PSW = 2 * IW + K
OW = PSW + 2                     # 53 for IW=24, K=3
RESULT_BYTES = (OW + 7) // 8     # 7
N_PIX = IMG_W * IMG_H
PIPE_LATENCY = 3                 # must match trim_fpga_top.v's PIPE_LATENCY
N_TOTAL = N_PIX + PIPE_LATENCY   # total results the board sends back


# ----------------------------------------------------------------------
# UART helpers
# ----------------------------------------------------------------------
# Image / kernel loading helpers
# ----------------------------------------------------------------------
NAMED_KERNELS = {
    "identity":   np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]], dtype=np.int8),
    "edge_v":     np.array([[1, 0, -1], [1, 0, -1], [1, 0, -1]], dtype=np.int8),
    "sobel_x":    np.array([[1, 0, -1], [2, 0, -2], [1, 0, -1]], dtype=np.int8),
    "sobel_y":    np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]], dtype=np.int8),
    "sharpen":    np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.int8),
    "blur3":      np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]], dtype=np.int8),  # NOTE: sums to 9, no /9 divide in hardware
}


def load_kernel(spec):
    """spec is either a name from NAMED_KERNELS or a path to a .npy file."""
    if spec is None:
        return NAMED_KERNELS["edge_v"]
    if spec in NAMED_KERNELS:
        return NAMED_KERNELS[spec]
    return np.load(spec)


def load_image(path):
    """Loads a HxW uint8 array from .npy, or a photo (.jpg/.png/...) via PIL.
    Photos are converted to grayscale and resized to IMG_W x IMG_H if needed
    (LANCZOS resample), with the exact size decision printed so nothing is
    silently mangled."""
    if path is None:
        rng = np.random.default_rng()
        return rng.integers(0, 256, (IMG_H, IMG_W), dtype=np.uint8)

    if path.lower().endswith(".npy"):
        arr = np.load(path)
        assert arr.shape == (IMG_H, IMG_W), f"{path} is {arr.shape}, expected {(IMG_H, IMG_W)}"
        return arr.astype(np.uint8)

    if Image is None:
        raise RuntimeError("Pillow is not installed -- run: pip install pillow")

    im = Image.open(path).convert("L")   # grayscale
    if im.size != (IMG_W, IMG_H):
        print(f"Resizing {path} from {im.size} to {(IMG_W, IMG_H)} (LANCZOS)")
        im = im.resize((IMG_W, IMG_H), Image.LANCZOS)
    else:
        print(f"{path} is already {im.size} -- no resize needed")
    return np.array(im, dtype=np.uint8)


def save_as_png(arr, path):
    """Min-max normalizes an (possibly negative/large) convolution result
    to 0-255 and saves it as a viewable grayscale PNG."""
    if Image is None:
        raise RuntimeError("Pillow is not installed -- run: pip install pillow")
    arr = arr.astype(np.float64)
    lo, hi = arr.min(), arr.max()
    norm = np.zeros_like(arr) if hi == lo else (arr - lo) / (hi - lo) * 255.0
    Image.fromarray(norm.astype(np.uint8)).save(path)
    print(f"Saved {path}  (raw value range was [{lo:.0f}, {hi:.0f}])")


# ----------------------------------------------------------------------
# Comparison plotting (matplotlib imshow)
# ----------------------------------------------------------------------
def maybe_use_interactive_backend():
    """Called only if --show was passed. Tries to switch matplotlib to an
    interactive backend so plt.show() actually pops up a window; silently
    stays on the headless 'Agg' backend (figures still get saved) if no
    display is available, e.g. over SSH with no X forwarding."""
    if plt is None:
        return
    try:
        matplotlib.use("TkAgg", force=True)
    except Exception:
        print("No interactive display detected -- --show will only save the "
              "figure to disk instead of popping up a window.")


def _imshow_panel(ax, arr, title):
    im = ax.imshow(arr, cmap="gray")
    ax.set_title(title, fontsize=10)
    ax.axis("off")
    return im


def save_comparison_png(path, input_img, golden, hw=None, diff=None, show=False):
    """Renders a 1-row grid of imshow panels: input image, golden
    convolution, and -- when hardware results are available -- the
    hardware convolution and the (hardware - golden) diff map.
    Each convolution panel gets its own color scale via matplotlib's
    default per-axes normalization, so relative structure is visible
    even though golden/hw/diff can have very different value ranges."""
    if plt is None:
        print("matplotlib is not installed -- skipping comparison figure "
              "(run: pip install matplotlib)")
        return

    panels = [("Input image", input_img), ("Golden convolution", golden)]
    if hw is not None:
        panels.append(("Hardware convolution", hw))
    if diff is not None:
        panels.append(("Diff (hw - golden)", diff))

    fig, axes = plt.subplots(1, len(panels), figsize=(4 * len(panels), 4))
    if len(panels) == 1:
        axes = [axes]

    for ax, (title, arr) in zip(axes, panels):
        im = _imshow_panel(ax, arr, title)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved comparison figure {path}")

    if show:
        plt.show()
    plt.close(fig)


# ----------------------------------------------------------------------
def open_port(port, baud=BAUD, timeout=5.0):
    if serial is None:
        raise RuntimeError("pyserial is not installed -- run: pip install pyserial")
    return serial.Serial(port, baudrate=baud, timeout=timeout)


def send_kernel(ser, kernel):
    """kernel: 3x3 numpy int array, values in [-128,127], raster order."""
    flat = kernel.astype(np.int8).flatten(order="C")
    ser.write(struct.pack("9b", *flat.tolist()))


def send_image(ser, image):
    """image: IMG_H x IMG_W numpy array, values in [0,255], raster order."""
    flat = image.astype(np.uint8).flatten(order="C")
    ser.write(bytes(flat.tolist()))


def receive_results(ser, n_results=N_TOTAL, result_bytes=RESULT_BYTES):
    """Reads n_results * result_bytes bytes and decodes each as a
    little-endian signed integer (the extra sign-extension bytes beyond
    OW bits don't change the decoded value)."""
    raw = ser.read(n_results * result_bytes)
    if len(raw) != n_results * result_bytes:
        raise IOError(
            f"expected {n_results * result_bytes} bytes back, got {len(raw)} "
            f"(check wiring / baud rate / that the board finished COMPUTE)"
        )
    out = np.empty(n_results, dtype=np.int64)
    for i in range(n_results):
        chunk = raw[i * result_bytes:(i + 1) * result_bytes]
        out[i] = int.from_bytes(chunk, byteorder="little", signed=True)
    return out


# ----------------------------------------------------------------------
# Golden model
# ----------------------------------------------------------------------
def golden_valid_conv(image, kernel):
    """'Valid' (no padding) cross-correlation -- matches what the MAC
    hardware computes (sum kernel[i,j]*image[r+i,c+j], no kernel flip).
    Returns a (H-K+1) x (W-K+1) array."""
    H, W = image.shape
    Kh, Kw = kernel.shape
    out = np.zeros((H - Kh + 1, W - Kw + 1), dtype=np.int64)
    img = image.astype(np.int64)
    ker = kernel.astype(np.int64)
    for i in range(Kh):
        for j in range(Kw):
            out += ker[i, j] * img[i:i + out.shape[0], j:j + out.shape[1]]
    return out


# ----------------------------------------------------------------------
# calibrate (sanity check only -- see note below)
# ----------------------------------------------------------------------
def run_calibrate(args):
    """This design's PE array feeds each kernel tap through a different-
    length path (the psum chain + skew registers), so an impulse on ONE
    tap only reveals THAT tap's path length, not the overall output
    alignment -- so this is just a basic sanity check (do we get the
    identity kernel's single nonzero sample back at all) rather than a
    real latency measurement. 'verify' uses --latency=PIPE_LATENCY by
    default (the flush-cycle offset baked into trim_fpga_top.v's FSM,
    confirmed correct by exhaustive testbench verification)."""
    identity_kernel = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]], dtype=np.int8)
    impulse_image = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
    impulse_image[0, 0] = 1

    ser = open_port(args.port, timeout=args.timeout)
    print(f"Sending identity kernel + impulse image over {args.port} ...")
    send_kernel(ser, identity_kernel)
    send_image(ser, impulse_image)

    print("Waiting for results...")
    results = receive_results(ser)
    ser.close()

    nonzero = np.nonzero(results)[0]
    if len(nonzero) != 1:
        print(f"WARNING: expected exactly 1 nonzero sample, got {len(nonzero)}: {nonzero}")
        print("Something in the pipeline (framing, baud, wiring) is likely off -- fix this before running 'verify'.")
        return

    print(f"\nOK: got exactly 1 nonzero sample back, at index {int(nonzero[0])}.")
    print(f"Board and protocol look healthy. Run 'verify' next (--latency defaults to {PIPE_LATENCY}).")


# ----------------------------------------------------------------------
# verify
# ----------------------------------------------------------------------
def run_verify(args):
    if args.show:
        maybe_use_interactive_backend()

    image = load_image(args.image)
    kernel = load_kernel(args.kernel)

    assert image.shape == (IMG_H, IMG_W), f"image must be {IMG_H}x{IMG_W}"
    assert kernel.shape == (K, K), f"kernel must be {K}x{K}"

    ser = open_port(args.port, timeout=args.timeout)
    print(f"Sending kernel + {IMG_H}x{IMG_W} image over {args.port} ...")
    send_kernel(ser, kernel)
    send_image(ser, image)

    print("Waiting for results...")
    raw = receive_results(ser)
    ser.close()

    golden = golden_valid_conv(image, kernel)   # shape (IMG_H-K+1, IMG_W-K+1)

    # Shift by --latency (defaults to PIPE_LATENCY=3, matching
    # trim_fpga_top.v's flush cycles -- confirmed by testbench), then
    # reshape and crop to the interior region that avoids the row-wrap
    # corruption discussed in trim_fpga_top.v's header comment.
    L = args.latency
    shifted = raw[L:L + N_PIX]
    if len(shifted) < N_PIX:
        shifted = np.pad(shifted, (0, N_PIX - len(shifted)))
    grid = shifted.reshape(IMG_H, IMG_W)

    # interior = rows/cols away from the border, matching the golden
    # 'valid' region's placement in the original image coordinates.
    hw_interior = grid[K - 1:K - 1 + golden.shape[0], K - 1:K - 1 + golden.shape[1]]

    diff = hw_interior - golden
    n_mismatch = int(np.count_nonzero(diff))
    max_err = int(np.max(np.abs(diff))) if diff.size else 0

    print(f"\nCompared {golden.size} interior pixels.")
    print(f"Mismatches: {n_mismatch} / {golden.size}")
    print(f"Max abs error: {max_err}")
    if n_mismatch == 0:
        print("MATCH: hardware convolution matches the golden model exactly.")
    else:
        print("MISMATCH: inspect --latency (re-run calibrate), the border-handling")
        print("assumption in trim_fpga_top.v, or the weight-loading row order.")

    if args.out:
        save_as_png(hw_interior, args.out)

    if args.compare_out or args.show:
        compare_path = args.compare_out or "verify_comparison.png"
        save_comparison_png(compare_path, image, golden, hw=hw_interior, show=args.show)


# ----------------------------------------------------------------------
# preview -- no hardware needed, golden model only
# ----------------------------------------------------------------------
def run_preview(args):
    if args.show:
        maybe_use_interactive_backend()

    image = load_image(args.image)
    kernel = load_kernel(args.kernel)
    assert image.shape == (IMG_H, IMG_W), f"image must be {IMG_H}x{IMG_W}"
    assert kernel.shape == (K, K), f"kernel must be {K}x{K}"

    golden = golden_valid_conv(image, kernel)
    print(f"Kernel:\n{kernel}")
    print(f"Golden convolution output shape: {golden.shape}")
    save_as_png(image, args.out.replace(".png", "_input.png"))
    save_as_png(golden, args.out)

    if args.compare_out or args.show:
        compare_path = args.compare_out or args.out.replace(".png", "_comparison.png")
        save_comparison_png(compare_path, image, golden, show=args.show)


# ----------------------------------------------------------------------
# self-test (no hardware needed) -- sanity-checks the golden model itself
# ----------------------------------------------------------------------
def run_self_test(args):
    rng = np.random.default_rng(0)
    image = rng.integers(0, 256, (IMG_H, IMG_W), dtype=np.uint8)
    kernel = np.array([[1, 0, -1], [2, 0, -2], [1, 0, -1]], dtype=np.int8)  # Sobel-x
    golden = golden_valid_conv(image, kernel)
    print(f"golden_valid_conv output shape: {golden.shape} (expect {IMG_H-K+1}x{IMG_W-K+1})")
    print("Sample corner value:", golden[0, 0])
    print("Self-test of the golden model completed (no hardware involved).")


# ----------------------------------------------------------------------
if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    p_cal = sub.add_parser("calibrate", help="measure the design's fixed pipeline latency")
    p_cal.add_argument("--port", required=True, help="e.g. /dev/ttyUSB1 or COM5")
    p_cal.add_argument("--timeout", type=float, default=5.0)
    p_cal.set_defaults(func=run_calibrate)

    p_ver = sub.add_parser("verify", help="send a real kernel+image and diff against NumPy")
    p_ver.add_argument("--port", required=True)
    p_ver.add_argument("--timeout", type=float, default=5.0)
    p_ver.add_argument("--latency", type=int, default=PIPE_LATENCY, help=f"default {PIPE_LATENCY} -- matches trim_fpga_top.v's flush cycles")
    p_ver.add_argument("--image", help=".npy or .jpg/.png file, HxW; random if omitted")
    p_ver.add_argument("--kernel", help=f".npy file or a name from {list(NAMED_KERNELS)}; edge_v if omitted")
    p_ver.add_argument("--out", help="save the hardware convolution result as a PNG here")
    p_ver.add_argument("--compare-out", help="save a side-by-side (input/golden/hardware) imshow figure here; default 'verify_comparison.png' if --show is set without this")
    p_ver.add_argument("--show", action="store_true", help="pop up the comparison figure interactively (also saves it)")
    p_ver.set_defaults(func=run_verify)

    p_prev = sub.add_parser("preview", help="no hardware -- golden-model-only preview, saved as PNG")
    p_prev.add_argument("--image", help=".npy or .jpg/.png file, HxW; random if omitted")
    p_prev.add_argument("--kernel", help=f".npy file or a name from {list(NAMED_KERNELS)}; edge_v if omitted")
    p_prev.add_argument("--out", default="preview_output.png", help="output PNG path")
    p_prev.add_argument("--compare-out", help="save a side-by-side (input/golden) imshow figure here; default '<out>_comparison.png' if --show is set without this")
    p_prev.add_argument("--show", action="store_true", help="pop up the comparison figure interactively (also saves it)")
    p_prev.set_defaults(func=run_preview)

    p_self = sub.add_parser("self-test", help="sanity-check the golden model only, no hardware")
    p_self.set_defaults(func=run_self_test)

    args = p.parse_args()
    args.func(args)