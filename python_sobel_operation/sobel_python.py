"""
=============================================================
  Sobel Edge Detection — Python (mirrors PIO-based Verilog)
-------------------------------------------------------------
  Input  : Physical (18).bmp
  Output : python_sobel_out.bmp

  This implementation replicates the exact pipeline stages
  used in the Verilog hardware design.

  Pipeline Stages:
    STAGE 0 : Data valid synchronization (edge detection)
    STAGE 1 : Line buffer update (3×3 window formation)
    STAGE 2 : Sobel convolution (sumX, sumY)
    STAGE 3 : Absolute value computation
    STAGE 4 : Gradient magnitude calculation
    STAGE 5 : Thresholding and output clamping

  Execution time measured using time.perf_counter()
=============================================================
"""

import struct
import time

# ── Configuration Parameters (aligned with Verilog design) ──
THRESHOLD = 0        # Minimum magnitude to consider as edge
GAIN      = 1        # Scaling factor for gradient magnitude

INPUT_FILE  = "Physical (18).bmp"
OUTPUT_FILE = "python_sobel_out.bmp"

# ── Sobel Kernels (Gradient operators) ─────────────────────
# Gx → Horizontal gradient
# Gy → Vertical gradient
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
    - pixel matrix
    - width
    - height
    """
    with open(filename, "rb") as f:
        raw = f.read()

    # Extract BMP header information
    px_off = struct.unpack_from("<I", raw, 10)[0]
    width  = struct.unpack_from("<i", raw, 18)[0]
    height = struct.unpack_from("<i", raw, 22)[0]
    bpp    = struct.unpack_from("<H", raw, 28)[0]

    # Handle top-down vs bottom-up BMP format
    flip = True
    if height < 0:
        height = -height
        flip   = False

    print(f"  BMP pixel offset : {px_off}")
    print(f"  Image dimensions : {width} x {height}, {bpp} bpp")

    # Row padding (BMP rows aligned to 4 bytes)
    pad      = (4 - (width % 4)) % 4
    row_size = width + pad

    # Initialize image buffer
    img = [[0] * width for _ in range(height)]

    # Read pixel data
    for row in range(height):
        src_row = (height - 1 - row) if flip else row
        offset  = px_off + src_row * row_size
        for col in range(width):
            img[row][col] = raw[offset + col]

    return img, width, height


# ============================================================
# BMP Writer (8-bit grayscale)
# ============================================================
def write_bmp(filename, img, width, height):
    """
    Writes a grayscale BMP image from pixel matrix
    """
    pad       = (4 - (width % 4)) % 4
    data_size = (width + pad) * height
    file_size = 54 + 256 * 4 + data_size
    px_offset = 54 + 256 * 4

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

        # Write pixel data (bottom-to-top format)
        pad_bytes = bytes(pad)
        for row in range(height - 1, -1, -1):
            f.write(bytes(img[row]))
            f.write(pad_bytes)


# ============================================================
# Signed 12-bit Conversion (matches Verilog behavior)
# ============================================================
def to_signed12(v):
    """
    Emulates 12-bit signed arithmetic used in hardware
    """
    v = v & 0xFFF
    return v - 0x1000 if v >= 0x800 else v


# ============================================================
# Pixel Processing Pipeline (Hardware-equivalent)
# ============================================================
def process_pixel(win, log=False):
    """
    Processes a single 3×3 window and computes output pixel.

    win → 3x3 neighborhood
    Returns:
        pixel_out
        stage-wise debug info (optional)
    """
    stages = {}

    # ── Stage 2: Sobel Convolution ──
    sumX = 0
    sumY = 0

    for i in range(3):
        for j in range(3):
            pixel = win[i][j]
            sumX += pixel * Gx[i][j]
            sumY += pixel * Gy[i][j]

    # Clamp to 12-bit signed (hardware equivalent)
    sumX = to_signed12(sumX)
    sumY = to_signed12(sumY)

    # ── Stage 3: Absolute Gradient ──
    absX = abs(sumX) & 0xFFF
    absY = abs(sumY) & 0xFFF

    # ── Stage 4: Magnitude ──
    magnitude = ((absX + absY) * GAIN) & 0x1FFF

    # ── Stage 5: Threshold + Saturation ──
    if magnitude > THRESHOLD:
        pixel_out = 255 if magnitude > 255 else magnitude
    else:
        pixel_out = 0

    return pixel_out, stages


# ============================================================
# Full Image Processing
# ============================================================
def sobel_edge_detection(input_img, width, height):
    """
    Applies Sobel edge detection over entire image
    """
    output_img = [[0] * width for _ in range(height)]

    # Process only valid interior pixels (ignore borders)
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            win = [
                [input_img[y-1][x-1], input_img[y-1][x], input_img[y-1][x+1]],
                [input_img[y  ][x-1], input_img[y  ][x], input_img[y  ][x+1]],
                [input_img[y+1][x-1], input_img[y+1][x], input_img[y+1][x+1]],
            ]
            pixel_out, _ = process_pixel(win)
            output_img[y][x] = pixel_out

    # Set borders to zero (no valid 3×3 window)
    for x in range(width):
        output_img[0][x] = 0
        output_img[height-1][x] = 0

    for y in range(height):
        output_img[y][0] = 0
        output_img[y][width-1] = 0

    return output_img


# ============================================================
# Main Execution
# ============================================================
def main():
    print("=" * 50)
    print("  Python Sobel Edge Detection")
    print("=" * 50)

    # Step 1: Read input image
    input_img, width, height = read_bmp(INPUT_FILE)

    # Step 2: Process image
    start = time.perf_counter()
    output_img = sobel_edge_detection(input_img, width, height)
    end = time.perf_counter()

    # Performance metrics
    elapsed = end - start
    print(f"\nExecution Time: {elapsed*1000:.3f} ms")

    # Step 3: Write output image
    write_bmp(OUTPUT_FILE, output_img, width, height)

    print("\nProcessing completed successfully.")


if __name__ == "__main__":
    main()
