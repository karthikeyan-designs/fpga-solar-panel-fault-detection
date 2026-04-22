"""
=============================================================
  Sobel Edge Detection — Python (mirrors PIO-based Verilog)
  ---------------------------------------------------------------
  Input  : Physical (18).bmp
  Output : python_sobel_out.bmp

  Pipeline stages match the Verilog always block exactly:

    STAGE 0 : 2-flop synchronizer  → rising-edge detection
    STAGE 1 : Shift line buffer     → load new column
    STAGE 2 : Compute sumX / sumY   → kernel convolution
    STAGE 3 : Absolute values       → absX, absY
    STAGE 4 : Magnitude             → (absX + absY) * GAIN
    STAGE 5 : Threshold + clamp     → pixel_out [7:0]

  Execution time measured with time.perf_counter()
=============================================================
"""

import struct
import time

# ── Parameters (identical to Verilog) ─────────────────────
THRESHOLD = 0
GAIN      = 1

INPUT_FILE  = "Physical (18).bmp"
OUTPUT_FILE = "python_sobel_out.bmp"

# ── Sobel Kernels (same as Verilog assign statements) ──────
#   Gx[i][j] / Gy[i][j] — signed 4-bit equivalent
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


# =============================================================
#  BMP Read — 8-bit grayscale
# =============================================================
def read_bmp(filename):
    with open(filename, "rb") as f:
        raw = f.read()

    px_off = struct.unpack_from("<I", raw, 10)[0]
    width  = struct.unpack_from("<i", raw, 18)[0]
    height = struct.unpack_from("<i", raw, 22)[0]
    bpp    = struct.unpack_from("<H", raw, 28)[0]

    flip = True
    if height < 0:
        height = -height
        flip   = False

    print(f"  BMP pixel offset : {px_off}")
    print(f"  Image dimensions : {width} x {height} pixels, {bpp} bpp")

    pad      = (4 - (width % 4)) % 4
    row_size = width + pad
    img      = [[0] * width for _ in range(height)]

    for row in range(height):
        src_row = (height - 1 - row) if flip else row
        offset  = px_off + src_row * row_size
        for col in range(width):
            img[row][col] = raw[offset + col]

    return img, width, height


# =============================================================
#  BMP Write — 8-bit grayscale
# =============================================================
def write_bmp(filename, img, width, height):
    pad       = (4 - (width % 4)) % 4
    data_size = (width + pad) * height
    file_size = 54 + 256 * 4 + data_size
    px_offset = 54 + 256 * 4  # 1078

    with open(filename, "wb") as f:
        # File header
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

        # Pixel data (bottom-to-top)
        pad_bytes = bytes(pad)
        for row in range(height - 1, -1, -1):
            f.write(bytes(img[row]))
            f.write(pad_bytes)


# =============================================================
#  Signed 12-bit clamp — mirrors Verilog reg signed [11:0]
# =============================================================
def to_signed12(v):
    v = v & 0xFFF
    return v - 0x1000 if v >= 0x800 else v


# =============================================================
#  Single-pixel pipeline — mirrors the Verilog always block
# =============================================================
def process_pixel(win, log=False):
    """
    win[i][j] = 3x3 neighbourhood, same indexing as line_buffer[i][j]
    i=0 -> top row, i=2 -> bottom row
    j=0 -> left col, j=2 -> right col
    Returns (pixel_out, stage_dict)
    """
    stages = {}

    # ── STAGE 0 : data_valid rising edge (2-flop synchronizer) ────
    # In hardware this gates the pipeline; in software we process
    # every interior pixel (equivalent to data_valid_rise = 1).
    if log:
        stages["S0_data_valid_rise"] = 1   # always 1 for interior pixels

    # ── STAGE 1 : Line buffer window ──────────────────────────────
    # Verilog shifts lb[][0..1], loads new column into lb[][2].
    # Here the 3x3 window IS the line buffer state at compute time.
    if log:
        stages["S1_line_buffer"] = [[win[i][j] for j in range(3)]
                                     for i in range(3)]

    # ── STAGE 2 : Compute sumX / sumY (kernel convolution) ────────
    # Verilog (blocking inside always):
    #   sumX = sumX + ({4'd0, line_buffer[i][j]} * Gx[i][j])
    #   sumY = sumY + ({4'd0, line_buffer[i][j]} * Gy[i][j])
    sumX = 0
    sumY = 0
    contrib_X = []
    contrib_Y = []
    for i in range(3):
        for j in range(3):
            pixel      = win[i][j]          # unsigned 8-bit
            cx         = pixel * Gx[i][j]
            cy         = pixel * Gy[i][j]
            sumX      += cx
            sumY      += cy
            contrib_X.append((i, j, pixel, Gx[i][j], cx))
            contrib_Y.append((i, j, pixel, Gy[i][j], cy))

    # Clip to signed 12-bit — mirrors reg signed [11:0]
    sumX = to_signed12(sumX)
    sumY = to_signed12(sumY)

    if log:
        stages["S2_contrib_X"] = contrib_X
        stages["S2_contrib_Y"] = contrib_Y
        stages["S2_sumX"]      = sumX
        stages["S2_sumY"]      = sumY

    # ── STAGE 3 : Absolute values ──────────────────────────────────
    # Verilog: absX_temp = (sumX < 0) ? -sumX : sumX
    #          absX = absX_temp[11:0]
    absX_temp = -sumX if sumX < 0 else sumX
    absY_temp = -sumY if sumY < 0 else sumY
    absX = absX_temp & 0xFFF
    absY = absY_temp & 0xFFF

    if log:
        stages["S3_absX_temp"] = absX_temp
        stages["S3_absY_temp"] = absY_temp
        stages["S3_absX"]      = absX
        stages["S3_absY"]      = absY

    # ── STAGE 4 : Magnitude ────────────────────────────────────────
    # Verilog: magnitude = (absX + absY) * GAIN   reg [12:0]
    magnitude = ((absX + absY) * GAIN) & 0x1FFF

    if log:
        stages["S4_magnitude"] = magnitude

    # ── STAGE 5 : Threshold + clamp → pixel_out_reg ───────────────
    # Verilog:
    #   if (magnitude > THRESHOLD)
    #       if (magnitude > 255)  pixel_out_reg <= 255
    #       else                  pixel_out_reg <= magnitude[7:0]
    #   else                      pixel_out_reg <= 0
    if magnitude > THRESHOLD:
        pixel_out = 255 if magnitude > 255 else magnitude
    else:
        pixel_out = 0

    if log:
        stages["S5_pixel_out"] = pixel_out

    return pixel_out, stages


# =============================================================
#  Stage-by-stage printer — detailed breakdown for one pixel
# =============================================================
def print_stage_log(row, col, stages):
    W = 56
    bar   = "─" * W
    blank = "│" + " " * W + "│"

    def line(text):
        print(f"│  {text:<{W-2}}│")

    print(f"┌{bar}┐")
    print(f"│{'  PIPELINE TRACE — Pixel (' + str(row) + ', ' + str(col) + ')':^{W}}│")
    print(f"├{bar}┤")
    print(blank)

    # ── Stage 0 ────────────────────────────────────────────────────
    line("STAGE 0 — 2-Flop Synchronizer (data_valid)")
    line("  Verilog: dv_ff1 <= data_valid; dv_ff2 <= dv_ff1;")
    line("           data_valid_rise = dv_ff1 & ~dv_ff2")
    line(f"  data_valid_rise = {stages['S0_data_valid_rise']}  → pipeline active")
    print(blank)

    # ── Stage 1 ────────────────────────────────────────────────────
    line("STAGE 1 — Line Buffer (3×3 window)")
    line("  Verilog: shift lb[][0..1], load new lb[][2]")
    line("  Current window (line_buffer[i][j]):")
    lb = stages["S1_line_buffer"]
    line(f"           col→  [0]  [1]  [2]")
    for i in range(3):
        lbl = ["row[0] top", "row[1] mid", "row[2] bot"][i]
        line(f"    {lbl}  {lb[i][0]:3d}  {lb[i][1]:3d}  {lb[i][2]:3d}")
    print(blank)

    # ── Stage 2 ────────────────────────────────────────────────────
    line("STAGE 2 — Kernel Convolution (sumX / sumY)")
    line("  Gx: [+1  0 -1]     Gy: [+1 +2 +1]")
    line("      [+2  0 -2]         [ 0  0  0]")
    line("      [+1  0 -1]         [-1 -2 -1]")
    print(blank)
    line("  Gx contributions (pixel × Gx[i][j]):")
    for (i, j, px, kx, cx) in stages["S2_contrib_X"]:
        if kx != 0:
            line(f"    [{i}][{j}] : {px:3d} × ({kx:+d}) = {cx:+5d}")
    line(f"  sumX (signed 12-bit) = {stages['S2_sumX']:+d}")
    print(blank)
    line("  Gy contributions (pixel × Gy[i][j]):")
    for (i, j, px, ky, cy) in stages["S2_contrib_Y"]:
        if ky != 0:
            line(f"    [{i}][{j}] : {px:3d} × ({ky:+d}) = {cy:+5d}")
    line(f"  sumY (signed 12-bit) = {stages['S2_sumY']:+d}")
    print(blank)

    # ── Stage 3 ────────────────────────────────────────────────────
    line("STAGE 3 — Absolute Values")
    line("  Verilog: absX_temp = (sumX < 0) ? -sumX : sumX")
    line("           absX = absX_temp[11:0]")
    line(f"  absX_temp = |{stages['S2_sumX']:+d}| = {stages['S3_absX_temp']}")
    line(f"  absY_temp = |{stages['S2_sumY']:+d}| = {stages['S3_absY_temp']}")
    line(f"  absX [11:0] = {stages['S3_absX']}")
    line(f"  absY [11:0] = {stages['S3_absY']}")
    print(blank)

    # ── Stage 4 ────────────────────────────────────────────────────
    line("STAGE 4 — Magnitude")
    line(f"  Verilog: magnitude = (absX + absY) * GAIN")
    line(f"         = ({stages['S3_absX']} + {stages['S3_absY']}) * {GAIN}")
    line(f"         = {stages['S4_magnitude']}  (13-bit reg)")
    print(blank)

    # ── Stage 5 ────────────────────────────────────────────────────
    line("STAGE 5 — Threshold & Clamp → pixel_out_reg")
    line(f"  THRESHOLD = {THRESHOLD},  GAIN = {GAIN}")
    mag = stages["S4_magnitude"]
    if mag > THRESHOLD:
        clamped = min(mag, 255)
        line(f"  {mag} > {THRESHOLD} → apply clamp:")
        line(f"    clamp({mag}, 0–255) = {clamped}")
    else:
        line(f"  {mag} ≤ {THRESHOLD} → pixel_out_reg = 0")
    line(f"  pixel_out = {stages['S5_pixel_out']}")
    print(blank)
    print(f"└{bar}┘")
    print()


# =============================================================
#  Full-image Sobel
# =============================================================
def sobel_edge_detection(input_img, width, height):
    output_img = [[0] * width for _ in range(height)]

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            win = [
                [input_img[y-1][x-1], input_img[y-1][x], input_img[y-1][x+1]],
                [input_img[y  ][x-1], input_img[y  ][x], input_img[y  ][x+1]],
                [input_img[y+1][x-1], input_img[y+1][x], input_img[y+1][x+1]],
            ]
            pixel_out, _ = process_pixel(win)
            output_img[y][x] = pixel_out

    # Border pixels = 0 (same as Verilog — window cannot be formed)
    for x in range(width):
        output_img[0][x]        = 0
        output_img[height-1][x] = 0
    for y in range(height):
        output_img[y][0]       = 0
        output_img[y][width-1] = 0

    return output_img


# =============================================================
#  MAIN
# =============================================================
def main():
    sep = "=" * 56
    print(sep)
    print("  Python Sobel — PIO Verilog Pipeline Mirror")
    print(sep)
    print()

    # ── 1. Read BMP ──────────────────────────────────────────
    print(f"[1] Reading input : {INPUT_FILE}")
    input_img, width, height = read_bmp(INPUT_FILE)
    print()

    # ── 2. Stage-by-stage demo on center pixel ───────────────
    demo_row = height // 2
    demo_col = width  // 2
    print(f"[2] Stage-by-stage pipeline trace for pixel ({demo_row}, {demo_col}):")
    win_demo = [
        [input_img[demo_row-1][demo_col-1], input_img[demo_row-1][demo_col], input_img[demo_row-1][demo_col+1]],
        [input_img[demo_row  ][demo_col-1], input_img[demo_row  ][demo_col], input_img[demo_row  ][demo_col+1]],
        [input_img[demo_row+1][demo_col-1], input_img[demo_row+1][demo_col], input_img[demo_row+1][demo_col+1]],
    ]
    _, demo_stages = process_pixel(win_demo, log=True)
    print_stage_log(demo_row, demo_col, demo_stages)

    # ── 3. Full image with timing ─────────────────────────────
    print(f"[3] Running Sobel on full image ({width}×{height})...")
    start = time.perf_counter()

    output_img = sobel_edge_detection(input_img, width, height)

    end        = time.perf_counter()
    elapsed_s  = end - start
    elapsed_ms = elapsed_s  * 1_000.0
    elapsed_us = elapsed_ms * 1_000.0

    print()
    print("=" * 45)
    print("         EXECUTION TIME RESULTS")
    print("=" * 45)
    print(f"  Time (s)       : {elapsed_s:.6f} s")
    print(f"  Time (ms)      : {elapsed_ms:.4f} ms")
    print(f"  Time (us)      : {elapsed_us:.1f} us")
    print(f"  Image size     : {width} × {height} px")
    print(f"  Total pixels   : {width * height:,}")
    if elapsed_s > 0:
        print(f"  Throughput     : {(width * height) / elapsed_s:,.0f} px/s")
    print("=" * 45)
    print()


    # ── 4. Write output BMP ──────────────────────────────────
    print(f"[4] Writing output : {OUTPUT_FILE}")
    write_bmp(OUTPUT_FILE, output_img, width, height)

    print()
    print("Done.")
    print(f"  Input  : {INPUT_FILE}")
    print(f"  Output : {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
