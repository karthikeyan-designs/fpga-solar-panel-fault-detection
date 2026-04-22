# FPGA Hardware Accelerator for Real-time Visual Inspection of Solar Panels  in Space Vehicles

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Setup](#hardware-setup)
- [Features](#features)
- [Hardware & Software Requirements](#hardware--software-requirements)
- [Repository Structure](#repository-structure)
- [Results](#results)
- [Team](#team)
- [Supervisor](#supervisor)

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

The system follows a **hardware–software co-design architecture**, where computationally intensive image processing is accelerated on FPGA, while control and decision-making are handled by the HPS (ARM processor).

### Architecture Overview

<p align="center">
  <img src="docs/block.png" width="600"/>
</p>
<p align="center"><em>Overall HPS–FPGA co-design architecture for solar panel defect detection</em></p>
### Data Flow

1. **Image Acquisition (HPS)**
   - Input solar panel image (.BMP)
   - Preprocessing: grayscale, resize, noise filtering

2. **HPS → FPGA Transfer**

<p align="center">
  <img src="docs/pd_interface.png" width="600"/>
</p>
<p align="center"><em>Platform Designer setup showing HPS–FPGA AXI interface and PIO connections</em></p>

The HPS communicates with the FPGA through the **AXI Lightweight bridge**, using memory-mapped PIO registers.

- `pixel_out_pio` → Sends pixel data from HPS to FPGA  
- `pixel_in_pio` → Reads processed edge data from FPGA  
- `data_valid_pio` → Synchronization signal for valid data transfer  

> Uses memory-mapped I/O via LW AXI bridge (e.g., base address 0xFF200000) for low-latency communication.
3. **Sobel Edge Detection (FPGA Implementation)**

<p align="center">
  <img src="docs/sobel_kernel.png" width="600"/>
</p>
<p align="center"><em>Sobel operator showing Gx, Gy computation and gradient magnitude</em></p>

- Line buffer generates 3×3 sliding window  
- Sobel convolution computes **Gx, Gy** using kernel multiplication and addition  
- Gradient magnitude ≈ |Gx| + |Gy|  
- Thresholding → Edge map (8-bit)

> These operations are implemented in FPGA because they are **pixel-level, computation-intensive tasks**.  
> The FPGA performs these operations in hardware using dedicated logic, enabling **faster execution compared to sequential CPU processing**.  
> This results in reduced execution time for image processing tasks such as Sobel edge detection.
4. **FPGA → HPS Return**
   - Edge-detected image transferred back

5. **HPS Post-processing**
   - Morphological filtering  
   - Feature extraction  
   - Crack & dust classification  

6. **Output**
   - Annotated image + defect label (Crack / Dust / Clean)
## Hardware Setup

<p align="center">
  <img src="docs/de1soc_setup.jpeg" width="400"/>
  <img src="docs/quartus_programmer.png" width="500"/>
</p>
The FPGA design is deployed on the DE1-SoC board, where the HPS communicates with the FPGA fabric to perform hardware-accelerated Sobel edge detection and defect analysis.  
The design is synthesized and programmed using Quartus Prime, and the successful configuration of the FPGA is verified through the Programmer tool.

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

## Usage (DE1-SoC)

> Execution is performed directly on the DE1-SoC (HPS Linux environment).

### 1. Mount USB Drive

```bash
ls /dev/sd*
mkdir -p /mnt/usb
mount -t vfat /dev/sda1 /mnt/usb
```
### 2. Copy Input Files
```
cp -r /mnt/input_realtime_images /home/root/
cp /mnt/fpga_multi_sobel_clean.c /home/root/
```
### 3. Compile on HPS
```
gcc -std=c99 -O1 -o phy fpga_multi_sobel_clean.c -lrt -lm
```
### 4. Run Programs
```
./sobel_dfftime
```
### 5. Copy Output to USB
```
cp /home/root/sobel_fifo_out.bmp /mnt/usb/
cp -r /home/root/fpga_sobel_realtime_mixed /mnt/usb/

ls /mnt/usb

sync
umount /mnt/usb
```
### Copy Files from USB to SD Card (HPS)

#### 1. Mount USB Drive

```bash
mkdir -p /mnt
mount /dev/sda1 /mnt
```
### 2. Verify Mounted Files
```
ls /mnt/usb
```
### 3. Copy Files to HPS (SD Card)
```
cp -r /mnt/input_realtime_images /home/root/
cp /mnt/fpga_multi_sobel_clean.c /home/root/
```
### 4. Unmount USB Drive
```
sync
umount /mnt
```


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

- **Karthikeyan S**  
- **Aparna S M**  
- **Muhammad Jamaldeen S**  
- **Phinehas Samuel S**

---

## Supervisor

- **Dr. A. Anitha Juliette** — Professor
---


