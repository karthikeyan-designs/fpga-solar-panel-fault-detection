/*
 * sobel_dfftime.c  -  FPGA Sobel via HPS with execution time measurement
 * 2-reg synchronizer: single pulse per column
 * Compile: gcc -std=c99 -O1 -o dffout_time sobel_dfftime.c -lrt
 */

#define _POSIX_C_SOURCE 199309L
#define _XOPEN_SOURCE   700

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>

#define IMG_W  512
#define IMG_H  512

#define LW_BRIDGE_BASE    0xFF200000UL
#define LW_BRIDGE_SPAN    0x00005000

#define DATA_VALID_OFFSET 0x00
#define PIXEL_IN_OFFSET   0x10
#define PIXEL_OUT_OFFSET  0x20

#define PULSE_ITERS  64
#define SETTLE_ITERS 128

#pragma pack(push,1)
typedef struct {
    uint16_t bfType;
    uint32_t bfSize;
    uint16_t bfReserved1;
    uint16_t bfReserved2;
    uint32_t bfOffBits;
    uint32_t biSize;
    int32_t  biWidth;
    int32_t  biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    int32_t  biXPelsPerMeter;
    int32_t  biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
} BMPHeader;
#pragma pack(pop)

static uint8_t input_img [IMG_H][IMG_W];
static uint8_t output_img[IMG_H][IMG_W];

int main(void)
{
    volatile int i;
    setbuf(stdout, NULL);
    printf("\n================ SOBEL HPS-FPGA START ================\n");

    /* ---------- READ INPUT BMP ---------- */
    FILE *fp = fopen("damage_solar.bmp", "rb");
    if (!fp) { perror("BMP open failed"); return -1; }

    BMPHeader header;
    if (fread(&header, sizeof(BMPHeader), 1, fp) != 1) {
        printf("ERROR: failed to read BMP header\n");
        fclose(fp); return -1;
    }

    printf("BMP: width=%d  height=%d  bpp=%d\n",
           header.biWidth, header.biHeight, header.biBitCount);

    if (header.biWidth       != IMG_W ||
        abs(header.biHeight) != IMG_H ||
        header.biBitCount    != 8) {
        printf("Unsupported BMP format\n");
        fclose(fp); return -1;
    }

    /* Save full header for output BMP */
    uint8_t *raw_header = (uint8_t *)malloc(header.bfOffBits);
    if (!raw_header) { printf("malloc failed\n"); fclose(fp); return -1; }
    fseek(fp, 0, SEEK_SET);
    if (fread(raw_header, 1, header.bfOffBits, fp) != header.bfOffBits) {
        printf("ERROR: failed to read BMP header bytes\n");
        fclose(fp); free(raw_header); return -1;
    }

    /* biHeight > 0 = bottom-up BMP: flip so input_img[0] = top row */
    fseek(fp, (long)header.bfOffBits, SEEK_SET);
    if (header.biHeight > 0) {
        for (int r = IMG_H - 1; r >= 0; r--)
            if (fread(input_img[r], 1, IMG_W, fp) != IMG_W) {
                printf("ERROR: failed to read row %d\n", r);
                fclose(fp); free(raw_header); return -1;
            }
    } else {
        for (int r = 0; r < IMG_H; r++)
            if (fread(input_img[r], 1, IMG_W, fp) != IMG_W) {
                printf("ERROR: failed to read row %d\n", r);
                fclose(fp); free(raw_header); return -1;
            }
    }
    fclose(fp);

    printf("  pixel[0][0]     = %u  (should be 37)\n", input_img[0][0]);
    printf("  pixel[100][100] = %u  (should be 41)\n", input_img[100][100]);

    /* ---------- MAP LW BRIDGE ---------- */
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem open failed"); free(raw_header); return -1; }

    volatile uint32_t *lw =
        (volatile uint32_t *)mmap(NULL, LW_BRIDGE_SPAN,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED, fd, LW_BRIDGE_BASE);
    if (lw == MAP_FAILED) {
        perror("mmap failed");
        close(fd); free(raw_header); return -1;
    }

    volatile uint32_t *data_valid = lw + (DATA_VALID_OFFSET / 4);
    volatile uint32_t *pixel_in   = lw + (PIXEL_IN_OFFSET   / 4);
    volatile uint32_t *pixel_out  = lw + (PIXEL_OUT_OFFSET  / 4);

    memset(output_img, 0, sizeof(output_img));
    *data_valid = 0;
    for (i = 0; i < 1000; i++);

    /* ======================================================
       HPS-FPGA CO-DESIGN EXECUTION TIME MEASUREMENT
       ====================================================== */
    struct timespec t_start, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    for (int y = 1; y < IMG_H - 1; y++) {
        for (int x = 1; x < IMG_W - 1; x++) {

            uint32_t packed =
                ((uint32_t)input_img[y-1][x] << 16) |
                ((uint32_t)input_img[y  ][x] <<  8) |
                 (uint32_t)input_img[y+1][x];

            *pixel_in   = packed;

            *data_valid = 1;
            for (i = 0; i < PULSE_ITERS;  i++);
            *data_valid = 0;
            for (i = 0; i < SETTLE_ITERS; i++);

            output_img[y][x] = (uint8_t)(*pixel_out & 0xFF);
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end);

    /* ---------- TIMING RESULTS ---------- */
    double exec_time =
        (double)(t_end.tv_sec  - t_start.tv_sec) +
        (double)(t_end.tv_nsec - t_start.tv_nsec) / 1e9;

    int total_pixels = (IMG_H - 2) * (IMG_W - 2);

    printf("\n--------------------------------------------------\n");
    printf("  FPGA Sobel Execution Time : %.3f ms\n",  exec_time * 1000.0);
    printf("  FPGA Sobel Execution Time : %.6f sec\n", exec_time);
    printf("  Total Pixels Processed    : %d\n",       total_pixels);
    printf("  Time per Pixel            : %.3f us\n",  exec_time * 1e6 / total_pixels);
    printf("--------------------------------------------------\n");

    /* ---------- COMPARE ROW 100 ---------- */
    int tb[] = {180, 124, 255, 255, 140, 255, 255,  72, 152,
                220, 232, 255, 255,  88, 255, 124,  80, 196, 252};

    printf("\nRow 100 comparison (cols 1-19):\n");
    printf("col |");
    for (int x = 1; x < 20; x++) printf(" %3d", x);
    printf("\nHPS |");
    for (int x = 1; x < 20; x++) printf(" %3d", output_img[100][x]);
    printf("\n TB |");
    for (int x = 0; x < 19; x++) printf(" %3d", tb[x]);
    printf("\n");

    int matches = 0;
    for (int x = 1; x < 20; x++)
        if (output_img[100][x] == tb[x-1]) matches++;
    printf("Matches: %d / 19\n", matches);
    if (matches == 19)
        printf("PASS: HPS output matches 2-reg TB reference exactly.\n");
    else
        printf("FAIL: %d mismatches.\n", 19 - matches);

    /* ---------- WRITE OUTPUT BMP ---------- */
    fp = fopen("fifo_damage1out.bmp", "wb");
    if (!fp) { perror("ERROR creating dffout_time.bmp"); }
    else {
        fwrite(raw_header, 1, header.bfOffBits, fp);
        if (header.biHeight > 0) {
            for (int r = IMG_H - 1; r >= 0; r--)
                fwrite(output_img[r], 1, IMG_W, fp);
        } else {
            for (int r = 0; r < IMG_H; r++)
                fwrite(output_img[r], 1, IMG_W, fp);
        }
        fclose(fp);
        printf("Output saved: dffout_time.bmp\n");
    }

    printf("================ SOBEL HPS-FPGA END ==================\n");

    *data_valid = 0;
    munmap((void *)lw, LW_BRIDGE_SPAN);
    close(fd);
    free(raw_header);
    return 0;
}