# FPGA Hardware Accelerator for Real-time Visual Inspection of Solar Panels  in Space Vehicles

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Hardware & Software Requirements](#hardware--software-requirements)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Results](#results)
- [Team](#team)
- [Supervisor](#supervisor)
- [License](#license)

---

## Overview

Solar panels in space vehicles are exposed to harsh environmental conditions that cause defects such as micro-cracks, dust accumulation, and delamination — all of which degrade power output and mission reliability. Manual or software-only inspection is too slow and resource-intensive for onboard, real-time use.

This project implements an **FPGA-based hardware accelerator** on the **DE1-SoC (Cyclone V)** that performs Sobel edge detection in hardware, with defect classification handled in C on the embedded ARM Cortex-A9 (HPS). The system achieves approximately **2× faster processing** compared to a CPU-only Python implementation.

**Defect classes detected:**
- 🔴 Physical damage (cracks / fractures)
- 🟡 Dust / surface contamination
- 🟢 Clean panel

---

## System Architecture

```
Camera (Satellite)
      │
      ▼ .BMP file
 ┌─────────────────────────────────────┐
 │  HPS — ARM Cortex-A9 (Linux)        │
 │  • Grayscale conversion             │
 │  • Gaussian blur + contrast boost   │
 │  • 512×512 resize & crop            │
 │  • Control & scheduling             │
 └────────────┬────────────────────────┘
              │ AXI Bridge (Lightweight)
              ▼
 ┌─────────────────────────────────────┐
 │  FPGA Fabric (Cyclone V)            │
 │  • Line buffer (3-row shift reg.)   │
 │  • Sobel convolution (Gx, Gy)       │
 │  • Gradient magnitude ≈ |Gx|+|Gy|  │
 │  • Thresholding → 8-bit edge map    │
 └────────────┬────────────────────────┘
              │ Edge map via AXI
              ▼
 ┌─────────────────────────────────────┐
 │  HPS — Post-processing & classify   │
 │  • Adaptive thresholding            │
 │  • Morphological filtering          │
 │  • Skeletonisation (Zhang-Suen)     │
 │  • Feature extraction               │
 │  • Rule-based classifier            │
 └─────────────────────────────────────┘
              │
              ▼
   Output: annotated BMP + fault label
```

---

## Features

- Sobel edge detection implemented in **Verilog** on FPGA for deterministic, parallel execution
- **HPS–FPGA communication** via memory-mapped PIO registers over AXI Lightweight bridge
- Double flip-flop synchroniser for safe cross-clock-domain signal handling
- Parallel crack and dust classification pipelines running on the HPS
- Composite annotated output image with severity label, bounding box, and heatmap
- **~2× faster** than CPU (Python) implementation on 512×512 images

---

## Hardware & Software Requirements

| Component | Details |
|-----------|---------|
| FPGA Board | Terasic DE1-SoC (Intel Cyclone V, ARM Cortex-A9) |
| FPGA IDE | Intel Quartus Prime (with Platform Designer) |
| HPS OS | Embedded Linux (ARM) |
| HPS Compiler | `arm-linux-gnueabihf-gcc` |
| Verification | MATLAB R2022+ |
| Image format | BMP (512×512, 8-bit grayscale) |

---

## Repository Structure

```
fpga-solar-panel-fault-detection/
├── README.md
├── LICENSE
├── .gitignore
├── report.pdf                        # Full project report
│
├── fpga/                             # Verilog source & Quartus project
│   ├── sobel_edge_detect.v           # Top-level Sobel module
│   ├── sobel_tb.v                    # Verilog testbench
│   ├── platform_designer.qsys        # Platform Designer system
│   └── quartus_project.qpf           # Quartus project file
│
├── hps/                              # C code running on ARM HPS
│   ├── main.c                        # Pipeline entry point
│   ├── classifier.c / classifier.h   # Crack, dust, clean logic
│   ├── bmp_utils.c / bmp_utils.h     # BMP read/write helpers
│   └── Makefile                      # Cross-compilation Makefile
│
├── matlab/                           # Reference & verification scripts
│   ├── sobel_reference.m             # MATLAB Sobel reference model
│   └── hex_to_bmp.m                  # Convert testbench HEX → BMP
│
├── dataset/                          # Sample solar panel images
│   ├── clean/
│   ├── dust/
│   └── physical_damage/
│
├── results/                          # Output images & performance data
│   ├── edge_fpga_*.bmp
│   └── performance_comparison.png
│
└── docs/                             # Diagrams and figures
    ├── block_diagram.png
    └── flowchart.png
```

> **Note:** The `dataset/` folder contains only sample images. The full 16-image ground-truth set used for validation is not included due to size. Place your own `.bmp` images in the respective subfolders before running.

---

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/<your-username>/fpga-solar-panel-fault-detection.git
cd fpga-solar-panel-fault-detection
```

### 2. Program the FPGA

1. Open `fpga/quartus_project.qpf` in **Quartus Prime**
2. Run **Platform Designer** to regenerate the system if needed
3. Compile the project (Processing → Start Compilation)
4. Program the DE1-SoC via JTAG using the Programmer tool

### 3. Build the HPS binary

Cross-compile on your host machine:

```bash
cd hps/
make
```

This produces a `sobel_defftime` binary compiled with `arm-linux-gnueabihf-gcc`.

### 4. Transfer and run on DE1-SoC

```bash
# Copy binary and dataset to the board
scp sobel_defftime root@<DE1-SoC-IP>:~/
scp -r ../dataset/ root@<DE1-SoC-IP>:~/

# SSH into the board and run
ssh root@<DE1-SoC-IP>
./sobel_defftime
```

Output images are written to `output_results/` on the board's filesystem.

### 5. Verify with MATLAB (optional)

Open `matlab/sobel_reference.m` and point it to any test image to compare MATLAB Sobel output against the FPGA-generated result. Use `matlab/hex_to_bmp.m` to convert testbench `.hex` output to a viewable BMP.

---

## Results

### Classification Accuracy

| Fault Class | Correctly Classified | Total | Accuracy |
|-------------|----------------------|-------|----------|
| Dust | 4 | 5 | 80% |
| Physical Damage | 5 | 5 | 100% |
| Clean | 4 | 5 | 80% |
| **Overall** | **13** | **15** | **86.67%** |

### Performance Comparison

| Method | Hardware | 1 Image (ms) | 5 Images (ms) |
|--------|----------|-------------|--------------|
| CPU (Python) | AMD Ryzen 3 7320U, 8 GB RAM | 1927.79 | 9379.92 |
| CPU (Python) | AMD Ryzen 7 6800H, 16 GB RAM | 1259.83 | 6080.18 |
| **HPS–FPGA** | **DE1-SoC (50 MHz FPGA, 800 MHz ARM A9)** | **981.25** | **4906.65** |

The FPGA implementation achieves approximately **2× speedup** over CPU implementations by exploiting parallel convolution and gradient computation in hardware.

---

## Team

| Name | Roll Number |
|------|-------------|
| Karthikeyan S | 311122106001 |
| Aparna S M | 311122106009 |
| Muhammad Jamaldeen S | 311122106043 |
| Phinehas Samuel S | 311122106046 |

**Department of Electronics and Communication Engineering**
Loyola–ICAM College of Engineering & Technology (LICET), Chennai — 600034
Anna University, April 2026

---

## Supervisor

**Dr. A. Anitha Juliette** — Professor, Dept. of ECE, LICET
**Ms. L. J. Jenifer Suriya** — Assistant Professor & Head, Dept. of ECE, LICET

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
