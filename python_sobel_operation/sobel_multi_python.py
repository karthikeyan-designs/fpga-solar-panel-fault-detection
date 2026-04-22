"""
=============================================================
  Sobel Edge Detection — Multi-Image Batch (PIO Verilog Mirror)
---------------------------------------------------------------
  Input  folder : input_physical_images/   (all .bmp files)
  Output folder : python_multi_sobel_output/

  This implementation mirrors the Verilog-based Sobel pipeline
  and applies it to all BMP images in the input directory.

  Pipeline stages:
    STAGE 0 : Data-valid synchronization / edge trigger
    STAGE 1 : 3×3 line-buffer window formation
    STAGE 2 : Sobel convolution (sumX, sumY)
    STAGE 3 : Absolute value computation
    STAGE 4 : Gradient magnitude calculation
    STAGE 5 : Thresholding and output clamping

  Execution time is measured per image and summarized at the end.
=============================================================
"""

import struct
import time
import os
import glob

# ── Configuration Parameters (aligned with Verilog design) ──
THRESHOLD = 0   # Minimum gradient magnitude required to mark an edge
GAIN      = 1   # Scaling factor applied to computed magnitude

INPUT_FOLDER  = "input_physical_images"
OUTPUT_FOLDER = "python_multi_sobel_output"

# ── Sobel Kernels ────────────────────────────────────────────
# Gx detects horizontal intensity change
# Gy detects vertical intensity change
Gx = [
    [ 1,  0, -1],
    [ 2,  0, -2],
    [ 1,  0, -1]
]

Gy = [
    [ 1,  2,  1],
    [ 0,  0,  0],
    [-1, -2, -1]
]


# ============================================================
# BMP Reader (8-bit grayscale)
# ============================================================
def read_bmp(filename):
    """
    Reads an 8-bit grayscale BMP image and returns:
    - image pixel matrix
    - width
    - height
    - bits per pixel
    """
    with open(filename, "rb") as f:
        raw = f.read()

    # Extract header fields from BMP file
    px_off = struct.unpack_from("<I", raw, 10)[0]
    width  = struct.unpack_from("<i", raw, 18)[0]
    height = struct.unpack_from("<i", raw, 22)[0]
    bpp    = struct.unpack_from("<H", raw, 28)[0]

    # Handle top-down BMP format if height is negative
    flip = True
    if height < 0:
        height = -height
        flip   = False

    # BMP rows are padded to 4-byte boundaries
    pad      = (4 - (width % 4)) % 4
    row_size = width + pad

    # Allocate image buffer
    img = [[0] * width for _ in range(height)]

    # Read pixel data into 2D array
    for row in range(height):
        src_row = (height - 1 - row) if flip else row
        offset  = px_off + src_row * row_size
        for col in range(width):
            img[row][col] = raw[offset + col]

    return img, width, height, bpp


# ============================================================
# BMP Writer (8-bit grayscale)
# ============================================================
def write_bmp(filename, img, width, height):
    """
    Writes an 8-bit grayscale BMP image from a 2D pixel array
    """
    pad       = (4 - (width % 4)) % 4
    data_size = (width + pad) * height
    file_size = 54 + 256 * 4 + data_size
    px_offset = 54 + 256 * 4

    with open(filename, "wb") as f:
        # BMP file header
        f.write(b"BM")
        f.write(struct.pack("<I",  file_size))
        f.write(struct.pack("<HH", 0, 0))
        f.write(struct.pack("<I",  px_offset))

        # DIB header
        f.write(struct.pack("<I",  40))
        f.write(struct.pack("<i",  width))
        f.write(struct.pack("<i",  height))
        f.write(struct.pack("<HH", 1, 8))
        f.write(struct.pack("<I",  0))
        f.write(struct.pack("<I",  data_size))
        f.write(struct.pack("<ii", 2835, 2835))
        f.write(struct.pack("<II", 0, 0))

        # Grayscale palette
        for i in range(256):
            f.write(bytes([i, i, i, 0]))

        # Pixel data written bottom-to-top
        pad_bytes = bytes(pad)
        for row in range(height - 1, -1, -1):
            f.write(bytes(img[row]))
            f.write(pad_bytes)


# ============================================================
# Signed 12-bit Conversion (matches Verilog behavior)
# ============================================================
def to_signed12(v):
    """
    Emulates signed 12-bit arithmetic used in hardware
    """
    v = v & 0xFFF
    return v - 0x1000 if v >= 0x800 else v


# ============================================================
# Single-Pixel Pipeline Processing
# ============================================================
def process_pixel(win, log=False):
    """
    Processes a single 3×3 window and returns:
    - output edge pixel
    - optional stage-wise debug data
    """
    stages = {}

    # Stage 0: Simulated data_valid trigger
    if log:
        stages["S0_data_valid_rise"] = 1

    # Stage 1: Capture line-buffer window for debugging
    if log:
        stages["S1_line_buffer"] = [[win[i][j] for j in range(3)]
                                    for i in range(3)]

    # Stage 2: Sobel convolution
    sumX = 0
    sumY = 0
    contrib_X = []
    contrib_Y = []

    for i in range(3):
        for j in range(3):
            pixel = win[i][j]
            cx    = pixel * Gx[i][j]
            cy    = pixel * Gy[i][j]
            sumX += cx
            sumY += cy
            contrib_X.append((i, j, pixel, Gx[i][j], cx))
            contrib_Y.append((i, j, pixel, Gy[i][j], cy))

    # Match Verilog-style signed 12-bit accumulation
    sumX = to_signed12(sumX)
    sumY = to_signed12(sumY)

    if log:
        stages["S2_contrib_X"] = contrib_X
        stages["S2_contrib_Y"] = contrib_Y
        stages["S2_sumX"]      = sumX
        stages["S2_sumY"]      = sumY

    # Stage 3: Absolute gradient values
    absX = (-sumX if sumX < 0 else sumX) & 0xFFF
    absY = (-sumY if sumY < 0 else sumY) & 0xFFF

    if log:
        stages["S3_absX"] = absX
        stages["S3_absY"] = absY

    # Stage 4: Gradient magnitude
    magnitude = ((absX + absY) * GAIN) & 0x1FFF

    if log:
        stages["S4_magnitude"] = magnitude

    # Stage 5: Thresholding and output saturation
    if magnitude > THRESHOLD:
        pixel_out = 255 if magnitude > 255 else magnitude
    else:
        pixel_out = 0

    if log:
        stages["S5_pixel_out"] = pixel_out

    return pixel_out, stages


# ============================================================
# Pipeline Trace Printer
# ============================================================
def print_stage_log(img_name, row, col, stages):
    """
    Prints stage-by-stage processing details for one reference pixel
    """
    W   = 58
    bar = "─" * W

    def line(text):
        print(f"│  {text:<{W-2}}│")

    print(f"┌{bar}┐")
    print(f"│{'  PIPELINE TRACE ─ ' + img_name + '  Pixel(' + str(row) + ',' + str(col) + ')':^{W}}│")
    print(f"├{bar}┤")
    print(f"│{' ' * W}│")

    line("STAGE 0 — 2-Flop Synchronizer (data_valid)")
    line("  data_valid_rise = 1  → pipeline active")
    print(f"│{' ' * W}│")

    line("STAGE 1 — Line Buffer (3×3 window captured)")
    lb = stages["S1_line_buffer"]
    line("           col→  [0]   [1]   [2]")
    for i in range(3):
        lbl = ["row[0] top", "row[1] mid", "row[2] bot"][i]
        line(f"    {lbl}  {lb[i][0]:3d}   {lb[i][1]:3d}   {lb[i][2]:3d}")
    print(f"│{' ' * W}│")

    line("STAGE 2 — Kernel Convolution")
    line("  Gx: [+1  0 -1]     Gy: [+1 +2 +1]")
    line("      [+2  0 -2]         [ 0  0  0]")
    line("      [+1  0 -1]         [-1 -2 -1]")
    print(f"│{' ' * W}│")

    line("  Gx contributions (pixel × Gx[i][j]):")
    for (i, j, px, kx, cx) in stages["S2_contrib_X"]:
        if kx != 0:
            line(f"    [{i}][{j}] : {px:3d} × ({kx:+d}) = {cx:+5d}")
    line(f"  sumX (signed 12-bit) = {stages['S2_sumX']:+d}")
    print(f"│{' ' * W}│")

    line("  Gy contributions (pixel × Gy[i][j]):")
    for (i, j, px, ky, cy) in stages["S2_contrib_Y"]:
        if ky != 0:
            line(f"    [{i}][{j}] : {px:3d} × ({ky:+d}) = {cy:+5d}")
    line(f"  sumY (signed 12-bit) = {stages['S2_sumY']:+d}")
    print(f"│{' ' * W}│")

    line("STAGE 3 — Absolute Values")
    line(f"  absX = |{stages['S2_sumX']:+d}| = {stages['S3_absX']}")
    line(f"  absY = |{stages['S2_sumY']:+d}| = {stages['S3_absY']}")
    print(f"│{' ' * W}│")

    line("STAGE 4 — Magnitude")
    line(f"  magnitude = ({stages['S3_absX']} + {stages['S3_absY']}) × GAIN({GAIN}) = {stages['S4_magnitude']}")
    print(f"│{' ' * W}│")

    line("STAGE 5 — Threshold & Clamp → pixel_out_reg")
    mag = stages["S4_magnitude"]
    if mag > THRESHOLD:
        line(f"  {mag} > THRESHOLD({THRESHOLD}) → clamp({mag}, 255) = {stages['S5_pixel_out']}")
    else:
        line(f"  {mag} ≤ THRESHOLD({THRESHOLD}) → pixel_out = 0")
    print(f"│{' ' * W}│")
    print(f"└{bar}┘")
    print()


# ============================================================
# Full-Image Sobel Processing
# ============================================================
def sobel_edge_detection(input_img, width, height):
    """
    Applies Sobel edge detection over the full image
    """
    output_img = [[0] * width for _ in range(height)]

    # Process only interior pixels with valid 3×3 neighborhoods
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            win = [
                [input_img[y-1][x-1], input_img[y-1][x], input_img[y-1][x+1]],
                [input_img[y  ][x-1], input_img[y  ][x], input_img[y  ][x+1]],
                [input_img[y+1][x-1], input_img[y+1][x], input_img[y+1][x+1]],
            ]
            pixel_out, _ = process_pixel(win)
            output_img[y][x] = pixel_out

    # Border pixels are set to zero because no full 3×3 window exists
    for x in range(width):
        output_img[0][x]        = 0
        output_img[height-1][x] = 0

    for y in range(height):
        output_img[y][0]        = 0
        output_img[y][width-1]  = 0

    return output_img


# ============================================================
# Per-Image Processing
# ============================================================
def process_one_image(idx, total, input_path, output_path):
    """
    Processes one BMP image and returns execution statistics
    """
    fname = os.path.basename(input_path)
    print(f"  [{idx}/{total}] {fname}")

    # Read input image
    input_img, width, height, bpp = read_bmp(input_path)
    print(f"         Size : {width} × {height} px, {bpp} bpp")

    # Generate stage trace using center pixel as reference
    dr = height // 2
    dc = width  // 2
    win_demo = [
        [input_img[dr-1][dc-1], input_img[dr-1][dc], input_img[dr-1][dc+1]],
        [input_img[dr  ][dc-1], input_img[dr  ][dc], input_img[dr  ][dc+1]],
        [input_img[dr+1][dc-1], input_img[dr+1][dc], input_img[dr+1][dc+1]],
    ]
    _, demo_stages = process_pixel(win_demo, log=True)
    print_stage_log(fname, dr, dc, demo_stages)

    # Execute Sobel processing and capture timing
    start      = time.perf_counter()
    output_img = sobel_edge_detection(input_img, width, height)
    end        = time.perf_counter()

    elapsed_s  = end - start
    elapsed_ms = elapsed_s * 1_000.0
    elapsed_us = elapsed_ms * 1_000.0
    throughput = (width * height) / elapsed_s if elapsed_s > 0 else 0

    print(f"         Time : {elapsed_ms:.4f} ms  ({elapsed_us:.1f} us)")
    print(f"   Throughput : {throughput:,.0f} px/s")

    # Write processed output image
    write_bmp(output_path, output_img, width, height)
    print(f"       Output : {os.path.basename(output_path)}")
    print()

    return {
        "file"      : fname,
        "width"     : width,
        "height"    : height,
        "pixels"    : width * height,
        "time_s"    : elapsed_s,
        "time_ms"   : elapsed_ms,
        "time_us"   : elapsed_us,
        "throughput": throughput,
    }


# ============================================================
# Batch Summary Printer
# ============================================================
def print_summary(results):
    """
    Prints per-image and overall batch performance summary
    """
    print("=" * 75)
    print("  BATCH SUMMARY")
    print("=" * 75)
    print(f"  {'#':<4} {'File':<30} {'Size':>12}  {'Time (ms)':>10}  {'px/s':>12}")
    print(f"  {'─'*4} {'─'*30} {'─'*12}  {'─'*10}  {'─'*12}")

    total_pixels = 0
    total_time_s = 0.0

    for i, r in enumerate(results, 1):
        size_str = f"{r['width']}×{r['height']}"
        print(f"  {i:<4} {r['file']:<30} {size_str:>12}  "
              f"{r['time_ms']:>10.4f}  {r['throughput']:>12,.0f}")
        total_pixels += r["pixels"]
        total_time_s += r["time_s"]

    total_ms = total_time_s * 1000.0
    avg_ms   = total_ms / len(results) if results else 0
    overall_throughput = total_pixels / total_time_s if total_time_s > 0 else 0

    print(f"  {'─'*4} {'─'*30} {'─'*12}  {'─'*10}  {'─'*12}")
    print(f"  {'TOTAL':<4} {len(results)} images processed    "
          f"{'':>12}  {total_ms:>10.4f}  {'':>12}")
    print(f"  {'AVG':<4} per image                    "
          f"{'':>12}  {avg_ms:>10.4f}  {overall_throughput:>12,.0f}")
    print("=" * 75)
    print(f"  Total pixels processed : {total_pixels:,}")
    print(f"  Total wall-clock time  : {total_time_s:.6f} s  ({total_ms:.4f} ms)")
    print("=" * 75)


# ============================================================
# Main Execution
# ============================================================
def main():
    """
    Batch Sobel processing entry point
    """
    print("=" * 65)
    print("  Python Sobel Batch — PIO Verilog Pipeline Mirror")
    print("=" * 65)
    print()

    # Check whether input folder exists
    if not os.path.isdir(INPUT_FOLDER):
        print(f"ERROR: Input folder not found: '{INPUT_FOLDER}'")
        print("  Create it and place your .bmp files inside.")
        return

    # Create output folder if it does not already exist
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    print(f"  Input  folder : {INPUT_FOLDER}/")
    print(f"  Output folder : {OUTPUT_FOLDER}/")
    print()

    # Collect all BMP files from the input directory
    bmp_files = sorted(
        glob.glob(os.path.join(INPUT_FOLDER, "*.bmp")) +
        glob.glob(os.path.join(INPUT_FOLDER, "*.BMP"))
    )

    # Remove duplicate matches if any
    bmp_files = list(dict.fromkeys(bmp_files))

    if not bmp_files:
        print(f"ERROR: No .bmp files found in '{INPUT_FOLDER}/'")
        return

    print(f"  Found {len(bmp_files)} BMP file(s) to process:")
    for f in bmp_files:
        print(f"    • {os.path.basename(f)}")
    print()

    # Process each image individually
    results = []
    total   = len(bmp_files)

    for idx, input_path in enumerate(bmp_files, 1):
        fname       = os.path.basename(input_path)
        output_path = os.path.join(OUTPUT_FOLDER, fname)

        try:
            stats = process_one_image(idx, total, input_path, output_path)
            results.append(stats)
        except Exception as e:
            print(f"  [ERROR] Skipping {fname}: {e}")
            print()

    # Print overall batch summary
    if results:
        print_summary(results)

    print()
    print("Done. All outputs saved to:", OUTPUT_FOLDER + "/")


if __name__ == "__main__":
    main()
