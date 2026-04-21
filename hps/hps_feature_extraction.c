/* hps_feature_extraction.c
 * Classifies 512x512 8-bit Sobel BMP images as CLEAN, DUST, or CRACK.
 * Input  : INPUT_DIR or a single BMP path from argv
 * Output : annotated BMP saved in OUTPUT_DIR
 */
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <dirent.h>
#include <sys/stat.h>

/* Image size */
#define W  512
#define H  512

/* Input/output settings */
#define INPUT_DIR   "fpga_sobel_realtime_mixed"
#define OUTPUT_DIR  "real_extraction_output"
#define MAX_IMAGES  300

/* Verdict thresholds
 * CLEAN: low std + few blobs
 * CRACK: high mean or high std
 * DUST : remaining cases
 */
#define VERDICT_CLEAN_STD_MAX       42.0f
#define VERDICT_CLEAN_BLOB_MAX      150
#define VERDICT_STD_THRESH          85.5f
#define VERDICT_BLOB_THRESH         1000
#define VERDICT_ABS_FLOOR           30.0f
/* Crack mean threshold */
#define VERDICT_MEAN_CRACK_MIN      80.0f   /* mean >= this -> CRACK */

typedef enum { V_CLEAN=0, V_DUST=1, V_CRACK=2 } Verdict;

/* Dust dashboard dimensions */
#define D_OUT_W     (W*2)
#define D_HDR_H     34
#define D_INFO_H    34
#define D_LEG_H     104
#define D_OUT_H     (D_HDR_H + H + D_INFO_H + D_LEG_H)
#define D_ROW_HDR   0
#define D_ROW_BODY  D_HDR_H
#define D_ROW_INFO  (D_HDR_H + H)
#define D_ROW_LEG   (D_HDR_H + H + D_INFO_H)

/* Dust pipeline tuning */
#define D_BORDER         20
#define D_EDGE_PCTILE    0.55f
#define D_LINE_RUN_MIN   22
#define D_DWIN           21
#define D_DENSITY_SIGMA  1.0f
#define D_DILATE         3
#define D_ERODE          1
#define D_AREA_MIN       80
#define D_AREA_MAX       15000
#define D_AR_MAX         4.5f
#define D_FILL_MIN       0.08f
#define D_MAX_REGIONS    1200
#define D_SEV_HIGH       5
#define D_SEV_MED        2
#define D_MAX_LABELS     16

/* Crack dashboard dimensions */
#define C_ADAPT_BLOCK  15
#define C_ADAPT_C      20
#define C_NOISE_NBR     3
#define C_BORDER       60
#define C_MAX_DIST    130
#define C_SEED_RADIUS  55
#define C_MIN_FRAG      5
#define C_BRIDGE        3
#define C_GROW_ITERS   20
#define C_DILATE_PX     2
#define C_IMPACT_RING  30
#define C_BG_BRIGHT    0.70f
#define C_SEV_HIGH    3000
#define C_SEV_MED     1000

/* Shared input buffer and verdict features */
static uint8_t raw[H][W];
static float   v_pixel_std;
static int     v_total_blobs;
static double  v_mean;

/* Dust pipeline buffers */
static uint8_t  d_bin  [H][W];
static uint8_t  d_grid [H][W];
static float    d_tex  [H][W];
static float    d_dens [H][W];
static uint8_t  d_mask [H][W];
static uint8_t  d_dil  [H][W];
static uint8_t  d_ero  [H][W];
static uint16_t d_lbl  [H][W];
static uint32_t d_integ[H+1][W+1];
static int      d_stk_x[H*W], d_stk_y[H*W];

typedef struct {
    int id,area,x0,y0,x1,y1,cx,cy;
    float ar,fill,score;
    int is_dust;
} DRegion;
static DRegion d_regs[D_MAX_REGIONS];
static int     d_nreg, d_dust_count;
static float   d_total_score;
static float   d_dmin,d_dmax,d_dmean,d_dstd;

/* Dust dashboard canvas */
static uint8_t d_outr[D_OUT_H][D_OUT_W];
static uint8_t d_outg[D_OUT_H][D_OUT_W];
static uint8_t d_outb[D_OUT_H][D_OUT_W];

/* Crack pipeline buffers */
static uint8_t  c_abin [H][W];
static uint8_t  c_thin [H][W];
static uint8_t  c_roi  [H][W];
static uint16_t c_lbl  [H][W];
static uint8_t  c_crack[H][W];
static uint8_t  c_thick[H][W];

#define C_MAX_REG 8000
typedef struct { int id,area,x0,x1,y0,y1; } CReg;
static CReg c_regs[C_MAX_REG];
static int  c_nreg;
static int  c_in_crack[C_MAX_REG];
static int  c_stk_x[H*W], c_stk_y[H*W];
static int  c_seed_cy, c_seed_cx;
static int  c_bbox_x0,c_bbox_x1,c_bbox_y0,c_bbox_y1;
static int  c_crack_area;

/* Crack output canvas */
static uint8_t c_outr[H][W];
static uint8_t c_outg[H][W];
static uint8_t c_outb[H][W];

/* Clean output canvas */
static uint8_t cl_outr[H][W];
static uint8_t cl_outg[H][W];
static uint8_t cl_outb[H][W];

/* BMP input/output helpers */
static int read_bmp(const char *fn){
    FILE *fp=fopen(fn,"rb");
    if(!fp){printf("  ERROR: %s\n",fn);return 0;}
    uint32_t off=0; int32_t bw=0,bh=0; uint16_t bbpp=0;
    fseek(fp,10,SEEK_SET); fread(&off,4,1,fp);
    fseek(fp,18,SEEK_SET); fread(&bw,4,1,fp); fread(&bh,4,1,fp);
    fseek(fp,28,SEEK_SET); fread(&bbpp,2,1,fp);
    int flip=(bh>0), ah=abs(bh);
    if(bw!=W||ah!=H||bbpp!=8){
        printf("  SKIP: %s (%dx%d %dbpp)\n",fn,bw,ah,bbpp);
        fclose(fp);return 0;
    }
    fseek(fp,(long)off,SEEK_SET);
    int pad=(4-(W%4))%4;
    for(int row=0;row<H;row++){
        int y=flip?(H-1-row):row;
        fread(raw[y],1,W,fp);
        fseek(fp,pad,SEEK_CUR);
    }
    fclose(fp);return 1;
}

/* Write 24-bit BMP for dust dashboard */
static void write_bmp24_dust(const char *fn){
    FILE *fp=fopen(fn,"wb");
    if(!fp){printf("  ERROR writing: %s\n",fn);return;}
    int rb=D_OUT_W*3, rp=(4-(rb%4))%4;
    int ds=(rb+rp)*D_OUT_H, fs=54+ds;
    uint8_t fh[14]={0x42,0x4D,
        (uint8_t)fs,(uint8_t)(fs>>8),(uint8_t)(fs>>16),(uint8_t)(fs>>24),
        0,0,0,0,54,0,0,0};
    fwrite(fh,1,14,fp);
    uint8_t dib[40]; memset(dib,0,40);
    dib[0]=40;
    dib[4]=(uint8_t)D_OUT_W; dib[5]=(uint8_t)(D_OUT_W>>8);
    dib[8]=(uint8_t)D_OUT_H; dib[9]=(uint8_t)(D_OUT_H>>8);
    dib[12]=1; dib[14]=24;
    fwrite(dib,1,40,fp);
    uint8_t z[4]={0};
    for(int y=D_OUT_H-1;y>=0;y--){
        for(int x=0;x<D_OUT_W;x++){
            fputc(d_outb[y][x],fp);
            fputc(d_outg[y][x],fp);
            fputc(d_outr[y][x],fp);
        }
        fwrite(z,1,rp,fp);
    }
    fclose(fp);
}

/* Write 24-bit BMP for crack dashboard */
static void write_bmp24_crack(const char *fn){
    FILE *fp=fopen(fn,"wb");
    if(!fp){printf("  ERROR writing: %s\n",fn);return;}
    int rb=W*3, rp=(4-(rb%4))%4;
    int ds=(rb+rp)*H, fs=54+ds;
    uint8_t fh[14]={0x42,0x4D,
        (uint8_t)fs,(uint8_t)(fs>>8),(uint8_t)(fs>>16),(uint8_t)(fs>>24),
        0,0,0,0,54,0,0,0};
    fwrite(fh,1,14,fp);
    uint8_t dib[40]; memset(dib,0,40);
    dib[0]=40;
    dib[4]=(uint8_t)W; dib[5]=(uint8_t)(W>>8);
    dib[8]=(uint8_t)H; dib[9]=(uint8_t)(H>>8);
    dib[12]=1; dib[14]=24;
    fwrite(dib,1,40,fp);
    uint8_t z[4]={0};
    for(int y=H-1;y>=0;y--){
        for(int x=0;x<W;x++){
            fputc(c_outb[y][x],fp);
            fputc(c_outg[y][x],fp);
            fputc(c_outr[y][x],fp);
        }
        fwrite(z,1,rp,fp);
    }
    fclose(fp);
}

/* Write 24-bit BMP for clean output */
static void write_bmp24_clean(const char *fn){
    FILE *fp=fopen(fn,"wb");
    if(!fp){printf("  ERROR writing: %s\n",fn);return;}
    int rb=W*3, rp=(4-(rb%4))%4;
    int ds=(rb+rp)*H, fs=54+ds;
    uint8_t fh[14]={0x42,0x4D,
        (uint8_t)fs,(uint8_t)(fs>>8),(uint8_t)(fs>>16),(uint8_t)(fs>>24),
        0,0,0,0,54,0,0,0};
    fwrite(fh,1,14,fp);
    uint8_t dib[40]; memset(dib,0,40);
    dib[0]=40;
    dib[4]=(uint8_t)W; dib[5]=(uint8_t)(W>>8);
    dib[8]=(uint8_t)H; dib[9]=(uint8_t)(H>>8);
    dib[12]=1; dib[14]=24;
    fwrite(dib,1,40,fp);
    uint8_t z[4]={0};
    for(int y=H-1;y>=0;y--){
        for(int x=0;x<W;x++){
            fputc(cl_outb[y][x],fp);
            fputc(cl_outg[y][x],fp);
            fputc(cl_outr[y][x],fp);
        }
        fwrite(z,1,rp,fp);
    }
    fclose(fp);
}

/* Compute image verdict using:
 * 1) global mean
 * 2) global standard deviation
 * 3) connected blob count above threshold
 */
static Verdict compute_verdict(void){
    /* Compute mean and standard deviation */
    double sum=0, sum2=0;
    int n=W*H;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        sum  += raw[y][x];
        sum2 += (double)raw[y][x]*raw[y][x];
    }
    v_mean = sum/n;
    double var = sum2/n - v_mean*v_mean;
    v_pixel_std = (float)sqrt(var<0?0:var);

    /* Count bright connected blobs */
    float abs_thresh = (float)(v_mean + v_pixel_std);
    if(abs_thresh < VERDICT_ABS_FLOOR) abs_thresh = VERDICT_ABS_FLOOR;

    static uint8_t visited[H][W];
    memset(visited,0,sizeof(visited));
    v_total_blobs = 0;

    for(int sy=0;sy<H;sy++) for(int sx=0;sx<W;sx++){
        if(raw[sy][sx] <= (uint8_t)abs_thresh || visited[sy][sx]) continue;
        int sp=0;
        d_stk_x[sp]=sx; d_stk_y[sp++]=sy;
        visited[sy][sx]=1;
        int size=0;
        while(sp>0){
            int cx=d_stk_x[--sp], cy=d_stk_y[sp];
            size++;
            for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++){
                int nx=cx+i, ny=cy+j;
                if(nx<0||nx>=W||ny<0||ny>=H) continue;
                if(visited[ny][nx]||raw[ny][nx]<=(uint8_t)abs_thresh) continue;
                visited[ny][nx]=1;
                d_stk_x[sp]=nx; d_stk_y[sp++]=ny;
            }
        }
        if(size>=5) v_total_blobs++;
    }

    /* Apply three-way decision logic */
    /* CLEAN: low variation and few blobs */
    if(v_pixel_std < VERDICT_CLEAN_STD_MAX &&
       v_total_blobs < VERDICT_CLEAN_BLOB_MAX)
        return V_CLEAN;

    /* CRACK: high mean or very high std */
    /* High mean helps catch crack images with moderate std */
    if((float)v_mean >= VERDICT_MEAN_CRACK_MIN ||
       v_pixel_std >= VERDICT_STD_THRESH)
        return V_CRACK;

    /* Otherwise classify as DUST */
    return V_DUST;
}

/* Clean panel render */
static const uint8_t CL_FONT[][5]={
{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
{0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
{0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
{0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x30,0x30,0x00,0x00},
{0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
{0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
{0x00,0x56,0x36,0x00,0x00},{0x00,0x08,0x14,0x22,0x41},{0x14,0x14,0x14,0x14,0x14},
{0x41,0x22,0x14,0x08,0x00},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
{0x3E,0x41,0x41,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
{0x7F,0x02,0x04,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
{0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
{0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
{0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
{0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x08,0x14,0x54,0x54,0x3C},
{0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
{0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
{0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
{0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
{0x44,0x64,0x54,0x4C,0x44}};

static void cl_dp(int y,int x,uint8_t r,uint8_t g,uint8_t b){
    if(y<0||y>=H||x<0||x>=W)return;
    cl_outr[y][x]=r; cl_outg[y][x]=g; cl_outb[y][x]=b;
}
static void cl_dc(int y,int x,char c,uint8_t r,uint8_t g,uint8_t b){
    if(c<' '||c>'z')return;
    const uint8_t *col=CL_FONT[(unsigned char)(c-' ')];
    for(int cx=0;cx<5;cx++){uint8_t bits=col[cx];
        for(int cy=0;cy<7;cy++) if(bits&(1<<cy)) cl_dp(y+cy,x+cx,r,g,b);}
}
static void cl_dt(int y,int x,const char *s,uint8_t r,uint8_t g,uint8_t b){
    for(;*s;s++,x+=6) cl_dc(y,x,*s,r,g,b);
}
static void cl_fill(int y0,int x0,int y1,int x1,uint8_t r,uint8_t g,uint8_t b){
    for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++) cl_dp(y,x,r,g,b);
}
static void cl_rect(int y0,int x0,int y1,int x1,int t,uint8_t r,uint8_t g,uint8_t b){
    for(int k=0;k<t;k++){
        for(int x=x0;x<=x1;x++){cl_dp(y0+k,x,r,g,b);cl_dp(y1-k,x,r,g,b);}
        for(int y=y0;y<=y1;y++){cl_dp(y,x0+k,r,g,b);cl_dp(y,x1-k,r,g,b);}
    }
}

static void cl_render(const char *fname){
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        uint8_t g=(uint8_t)(raw[y][x]*0.55f);
        cl_outr[y][x]=cl_outg[y][x]=cl_outb[y][x]=g;
    }
    cl_fill(4,4,54,W-4, 10,60,20);
    cl_rect(4,4,54,W-4, 2, 40,200,60);
    int ck_y=12, ck_x=16;
    for(int i=0;i<8;i++){
        cl_dp(ck_y+20+i,ck_x+i,   80,255,80);
        cl_dp(ck_y+20+i,ck_x+i+1, 80,255,80);
    }
    for(int i=0;i<18;i++){
        cl_dp(ck_y+28-i,ck_x+8+i,   80,255,80);
        cl_dp(ck_y+28-i,ck_x+8+i+1, 80,255,80);
    }
    cl_dt(14, 50, "SOLAR PANEL DEFECT REPORT", 180,255,180);
    cl_dt(26, 50, "Status   : CLEAN - No Defect Detected", 120,255,120);
    cl_dt(38, 50, "Action   : No maintenance required",    100,220,100);

    cl_fill(64,4,120,W-4, 0,0,0);
    cl_rect(64,4,120,W-4, 1, 60,60,60);
    cl_dt(69,  8, "PANEL HEALTH METRICS",         200,200,200);
    for(int x=6;x<W-4;x++) cl_dp(82,x,60,60,60);

    char buf[80];
    sprintf(buf,"Pixel Mean   : %.2f  (crack thresh %.1f)",
            (float)v_mean, VERDICT_MEAN_CRACK_MIN);
    cl_dt(86,  8, buf, 160,200,160);
    sprintf(buf,"Pixel StdDev : %.2f  (threshold %.1f)",
            v_pixel_std, VERDICT_CLEAN_STD_MAX);
    cl_dt(98,  8, buf, 160,200,160);
    sprintf(buf,"Blob Count   : %d  (threshold %d)",
            v_total_blobs, VERDICT_CLEAN_BLOB_MAX);
    cl_dt(110, 8, buf, 160,200,160);
    cl_dt(118, 8, fname, 100,100,100);

    cl_fill(H-18,0,H-1,W-1, 10,40,10);
    cl_dt(H-14, 8, "CLEAN: std and blob count both below clean thresholds",
          80,200,80);
}

/* Dust detection pipeline */
static void d_dp(int y,int x,uint8_t r,uint8_t g,uint8_t b){
    if(y>=0&&y<D_OUT_H&&x>=0&&x<D_OUT_W){
        d_outr[y][x]=r; d_outg[y][x]=g; d_outb[y][x]=b;
    }
}
static void d_fill(int y0,int x0,int y1,int x1,uint8_t r,uint8_t g,uint8_t b){
    for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++) d_dp(y,x,r,g,b);
}
static void d_hline(int y,int x0,int x1,uint8_t r,uint8_t g,uint8_t b){
    for(int x=x0;x<=x1;x++) d_dp(y,x,r,g,b);
}
static void d_blend(uint8_t *dr,uint8_t *dg,uint8_t *db,
                    uint8_t sr,uint8_t sg,uint8_t sb,float a){
    float ia=1.0f-a;
    *dr=(uint8_t)(*dr*ia+sr*a);
    *dg=(uint8_t)(*dg*ia+sg*a);
    *db=(uint8_t)(*db*ia+sb*a);
}
static void d_heat(float t,uint8_t *r,uint8_t *g,uint8_t *b){
    typedef struct{float p;uint8_t r,g,b;}S;
    static const S stops[]={
        {0.00f,5,10,30},{0.20f,0,40,120},
        {0.45f,0,160,230},{0.70f,255,200,0},{1.00f,255,30,0}};
    if(t<=0){*r=5;*g=10;*b=30;return;}
    if(t>=1){*r=255;*g=30;*b=0;return;}
    for(int i=1;i<5;i++) if(t<=stops[i].p){
        float f=(t-stops[i-1].p)/(stops[i].p-stops[i-1].p);
        *r=(uint8_t)(stops[i-1].r+f*(stops[i].r-stops[i-1].r));
        *g=(uint8_t)(stops[i-1].g+f*(stops[i].g-stops[i-1].g));
        *b=(uint8_t)(stops[i-1].b+f*(stops[i].b-stops[i-1].b));
        return;
    }
}
static void d_dustcol(float rel,uint8_t *r,uint8_t *g,uint8_t *b){
    if(rel>0.65f){*r=230;*g=40;*b=40;}
    else if(rel>0.30f){*r=255;*g=160;*b=0;}
    else{*r=80;*g=230;*b=70;}
}

#define FW 5
#define FH 7
static const uint8_t DFONT[42][FH]={
{0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
{0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},{0x1F,0x01,0x02,0x06,0x01,0x11,0x0E},
{0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},{0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
{0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},{0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
{0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},{0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
{0x04,0x0A,0x11,0x11,0x1F,0x11,0x11},{0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
{0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},{0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
{0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},{0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
{0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},{0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
{0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},{0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
{0x11,0x12,0x14,0x18,0x14,0x12,0x11},{0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
{0x11,0x1B,0x15,0x15,0x11,0x11,0x11},{0x11,0x19,0x15,0x13,0x11,0x11,0x11},
{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},{0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
{0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},{0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
{0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},{0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
{0x11,0x11,0x11,0x11,0x11,0x11,0x0E},{0x11,0x11,0x11,0x11,0x0A,0x0A,0x04},
{0x11,0x11,0x15,0x15,0x15,0x0A,0x0A},{0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
{0x11,0x11,0x0A,0x04,0x04,0x04,0x04},{0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
{0x00,0x00,0x00,0x00,0x00,0x00,0x04},{0x00,0x04,0x00,0x00,0x00,0x04,0x00},
{0x18,0x19,0x02,0x04,0x08,0x13,0x03},{0x01,0x02,0x02,0x04,0x08,0x08,0x10},
{0x00,0x00,0x00,0x1F,0x00,0x00,0x00},{0x00,0x00,0x00,0x00,0x00,0x00,0x1F}
};
static int d_cidx(char c){
    if(c>='0'&&c<='9')return c-'0';
    if(c>='A'&&c<='Z')return 10+(c-'A');
    if(c>='a'&&c<='z')return 10+(c-'a');
    if(c=='.')return 36; if(c==':')return 37; if(c=='%')return 38;
    if(c=='/')return 39; if(c=='-')return 40; if(c=='_')return 41;
    return -1;
}
static int d_dchar(int y,int x,char c,int sc,
                   uint8_t fr,uint8_t fg,uint8_t fb,
                   uint8_t br,uint8_t bg_,uint8_t bb){
    int idx=d_cidx(c);
    for(int row=0;row<FH;row++) for(int col=0;col<FW;col++){
        int on=(idx>=0)&&(DFONT[idx][row]&(0x10>>col));
        for(int sy=0;sy<sc;sy++) for(int sx=0;sx<sc;sx++)
            d_dp(y+row*sc+sy,x+col*sc+sx,on?fr:br,on?fg:bg_,on?fb:bb);
    }
    return x+(FW+1)*sc;
}
static int d_dstr(int y,int x,const char *s,int sc,
                  uint8_t fr,uint8_t fg,uint8_t fb,
                  uint8_t br,uint8_t bg_,uint8_t bb){
    while(*s){
        if(*s==' ')x+=(FW+1)*sc;
        else x=d_dchar(y,x,*s,sc,fr,fg,fb,br,bg_,bb);
        s++;
    }
    return x;
}
static void d_dstr_c(int y,int x0,int x1,const char *s,int sc,
                     uint8_t fr,uint8_t fg,uint8_t fb,
                     uint8_t br,uint8_t bg_,uint8_t bb){
    int tw=(int)strlen(s)*(FW+1)*sc;
    d_dstr(y,x0+(x1-x0-tw)/2,s,sc,fr,fg,fb,br,bg_,bb);
}

static int d_cmpf(const void*a,const void*b){
    float fa=*(float*)a,fb=*(float*)b;return(fa>fb)-(fa<fb);
}
static void d_build_binary(void){
    float vals[H*W]; int cnt=0;
    for(int y=D_BORDER;y<H-D_BORDER;y++)
        for(int x=D_BORDER;x<W-D_BORDER;x++)
            if(raw[y][x]>0) vals[cnt++]=(float)raw[y][x];
    qsort(vals,cnt,sizeof(float),d_cmpf);
    int idx=(int)(D_EDGE_PCTILE*cnt); if(idx>=cnt)idx=cnt-1;
    float thr=vals[idx];
    for(int y=D_BORDER;y<H-D_BORDER;y++)
        for(int x=D_BORDER;x<W-D_BORDER;x++)
            d_bin[y][x]=(raw[y][x]>=thr)?1:0;
}
static void d_suppress_grid(void){
    for(int y=0;y<H;y++){
        int rs=-1;
        for(int x=0;x<=W;x++){
            int v=(x<W)?d_bin[y][x]:0;
            if(v){if(rs<0)rs=x;}
            else{if(rs>=0&&x-rs>=D_LINE_RUN_MIN)for(int k=rs;k<x;k++)d_grid[y][k]=1;rs=-1;}
        }
    }
    for(int x=0;x<W;x++){
        int rs=-1;
        for(int y=0;y<=H;y++){
            int v=(y<H)?d_bin[y][x]:0;
            if(v){if(rs<0)rs=y;}
            else{if(rs>=0&&y-rs>=D_LINE_RUN_MIN)for(int k=rs;k<y;k++)d_grid[k][x]=1;rs=-1;}
        }
    }
    for(int y=0;y<H;y++) for(int x=0;x<W;x++)
        d_tex[y][x]=(d_bin[y][x]&&!d_grid[y][x])?1.0f:0.0f;
}
static uint32_t d_rsum(int x0,int y0,int x1,int y1){
    return d_integ[y1+1][x1+1]-d_integ[y0][x1+1]
          -d_integ[y1+1][x0]+d_integ[y0][x0];
}
static void d_build_density(void){
    memset(d_integ,0,sizeof(d_integ));
    for(int y=1;y<=H;y++){
        uint32_t rs=0;
        for(int x=1;x<=W;x++){rs+=(uint32_t)d_tex[y-1][x-1];d_integ[y][x]=d_integ[y-1][x]+rs;}
    }
    int half=D_DWIN/2;
    double sum=0,sum2=0; int cnt=0;
    for(int y=D_BORDER;y<H-D_BORDER;y++) for(int x=D_BORDER;x<W-D_BORDER;x++){
        int x0=x-half<0?0:x-half, x1=x+half>=W?W-1:x+half;
        int y0=y-half<0?0:y-half, y1=y+half>=H?H-1:y+half;
        int area=(x1-x0+1)*(y1-y0+1);
        float d=(float)d_rsum(x0,y0,x1,y1)/(float)area;
        d_dens[y][x]=d; sum+=d; sum2+=d*d; cnt++;
    }
    d_dmean=(float)(sum/cnt);
    float var=(float)(sum2/cnt-d_dmean*d_dmean); if(var<0)var=0; d_dstd=sqrtf(var);
    d_dmin=1e9f; d_dmax=-1e9f;
    for(int y=D_BORDER;y<H-D_BORDER;y++) for(int x=D_BORDER;x<W-D_BORDER;x++){
        if(d_dens[y][x]<d_dmin)d_dmin=d_dens[y][x];
        if(d_dens[y][x]>d_dmax)d_dmax=d_dens[y][x];
    }
    float thr=d_dmean+D_DENSITY_SIGMA*d_dstd;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++)
        d_mask[y][x]=(d_dens[y][x]>thr)?1:0;
}
static void d_morph(void){
    uint8_t tmp[H][W]; memcpy(tmp,d_mask,sizeof(tmp));
    for(int it=0;it<D_DILATE;it++){
        uint8_t cur[H][W]; memcpy(cur,tmp,sizeof(cur));
        for(int y=1;y<H-1;y++) for(int x=1;x<W-1;x++) if(!cur[y][x]){
            int s=0;for(int j=-1;j<=1;j++)for(int i=-1;i<=1;i++)s+=cur[y+j][x+i];
            if(s>0)tmp[y][x]=1;
        }
    }
    memcpy(d_dil,tmp,sizeof(d_dil));
    for(int it=0;it<D_ERODE;it++){
        uint8_t cur[H][W]; memcpy(cur,tmp,sizeof(cur));
        for(int y=1;y<H-1;y++) for(int x=1;x<W-1;x++) if(cur[y][x]){
            int s=0;for(int j=-1;j<=1;j++)for(int i=-1;i<=1;i++)s+=cur[y+j][x+i];
            if(s<9)tmp[y][x]=0;
        }
    }
    memcpy(d_ero,tmp,sizeof(d_ero));
}
static void d_cc_label(void){
    d_nreg=0; uint16_t cur=1; memset(d_lbl,0,sizeof(d_lbl));
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        if(!d_ero[y][x]||d_lbl[y][x]) continue;
        if(d_nreg>=D_MAX_REGIONS) continue;
        DRegion *r=&d_regs[d_nreg];
        r->id=cur;r->area=0;r->x0=99999;r->y0=99999;r->x1=0;r->y1=0;r->is_dust=0;r->score=0;
        int sp=0;d_stk_x[sp]=x;d_stk_y[sp++]=y;d_lbl[y][x]=cur;
        while(sp>0){
            int cx=d_stk_x[--sp],cy=d_stk_y[sp]; r->area++;
            if(cx<r->x0)r->x0=cx;if(cx>r->x1)r->x1=cx;
            if(cy<r->y0)r->y0=cy;if(cy>r->y1)r->y1=cy;
            for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++){
                int nx=cx+i,ny=cy+j;
                if(nx>=0&&nx<W&&ny>=0&&ny<H&&d_ero[ny][nx]&&!d_lbl[ny][nx]){
                    d_lbl[ny][nx]=cur;d_stk_x[sp]=nx;d_stk_y[sp++]=ny;
                }
            }
        }
        r->cx=(r->x0+r->x1)/2;r->cy=(r->y0+r->y1)/2;cur++;d_nreg++;
    }
}
static void d_classify(void){
    d_dust_count=0; d_total_score=0;
    for(int i=0;i<d_nreg;i++){
        DRegion *r=&d_regs[i];
        int bw=r->x1-r->x0+1,bh=r->y1-r->y0+1,bbox=bw*bh;
        r->ar=(bw>bh)?(float)bw/bh:(float)bh/bw;
        r->fill=(bbox>0)?(float)r->area/bbox:0;
        float s=0; int cnt=0;
        for(int y=r->y0;y<=r->y1;y++) for(int x=r->x0;x<=r->x1;x++)
            if(d_lbl[y][x]==r->id){s+=d_dens[y][x];cnt++;}
        r->score=(cnt>0)?(s/cnt):0;
        if(r->area<D_AREA_MIN||r->area>D_AREA_MAX) continue;
        if(r->ar>D_AR_MAX||r->fill<D_FILL_MIN)     continue;
        r->is_dust=1; d_dust_count++; d_total_score+=r->score;
    }
}
static int d_is_boundary(int y,int x){
    for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++){
        int ny=y+j,nx=x+i;
        if(ny>=0&&ny<H&&nx>=0&&nx<W&&!d_ero[ny][nx]) return 1;
    }
    return 0;
}

static void d_render(const char *filename){
    for(int y=0;y<D_OUT_H;y++) for(int x=0;x<D_OUT_W;x++){
        d_outr[y][x]=8;d_outg[y][x]=10;d_outb[y][x]=20;
    }
    for(int y=0;y<D_HDR_H;y++){
        uint8_t r=8+(y*5)/D_HDR_H,g=12+(y*7)/D_HDR_H,b=28+(y*16)/D_HDR_H;
        for(int x=0;x<D_OUT_W;x++) d_dp(y,x,r,g,b);
    }
    d_hline(D_HDR_H-1,0,D_OUT_W-1,70,90,150);
    d_dstr_c(10,0,D_OUT_W,"SOLAR PANEL DUST DETECTION",1,205,230,255,8,15,35);
    char fnbuf[96]; snprintf(fnbuf,sizeof(fnbuf),"FILE: %s",filename);
    int tw=(int)strlen(fnbuf)*(FW+1);
    int fx=D_OUT_W-8-tw; if(fx<D_OUT_W/2)fx=D_OUT_W/2;
    d_dstr(22,fx,fnbuf,1,140,150,175,8,15,35);

    float drange=(d_dmax-d_dmin>1e-6f)?(d_dmax-d_dmin):1.0f;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        uint8_t g2=(uint8_t)(raw[y][x]*0.22f);
        int oy=D_ROW_BODY+y;
        d_outr[oy][x]=g2;d_outg[oy][x]=g2;d_outb[oy][x]=g2;
        float t=(d_dens[y][x]-d_dmin)/drange;
        uint8_t hr=0,hg=0,hb=0; d_heat(t,&hr,&hg,&hb);
        d_blend(&d_outr[oy][x],&d_outg[oy][x],&d_outb[oy][x],hr,hg,hb,0.82f);
    }
    d_fill(D_ROW_BODY,0,D_ROW_BODY+18,W-1,8,12,28);
    d_hline(D_ROW_BODY+18,0,W-1,0,100,200);
    d_dstr_c(D_ROW_BODY+5,0,W,"EDGE DENSITY HEATMAP",1,100,180,255,8,12,28);
    float top5[5]={0}; int tx5[5]={0},ty5[5]={0};
    for(int y=D_BORDER;y<H-D_BORDER;y++) for(int x=D_BORDER;x<W-D_BORDER;x++){
        float d=d_dens[y][x];
        for(int k=0;k<5;k++) if(d>top5[k]){
            for(int j=4;j>k;j--){top5[j]=top5[j-1];tx5[j]=tx5[j-1];ty5[j]=ty5[j-1];}
            top5[k]=d;tx5[k]=x;ty5[k]=y;break;
        }
    }
    for(int k=0;k<5;k++){
        int py=D_ROW_BODY+ty5[k],px=tx5[k];
        for(int i=-6;i<=6;i++){d_dp(py,px+i,255,255,100);d_dp(py+i,px,255,255,100);}
    }

    float sc_base=d_dmean+D_DENSITY_SIGMA*d_dstd;
    float sc_rng=(d_dmax-sc_base>1e-6f)?(d_dmax-sc_base):1.0f;
    for(int y=0;y<H;y++){
        int oy=D_ROW_BODY+y;
        for(int x=0;x<W;x++){
            int ox=W+x;
            uint8_t g2=(uint8_t)(raw[y][x]*0.30f);
            d_outr[oy][ox]=g2;d_outg[oy][ox]=g2;d_outb[oy][ox]=g2;
            if(d_ero[y][x]&&d_lbl[y][x]){
                int idx=(int)d_lbl[y][x]-1;
                if(idx>=0&&idx<d_nreg&&d_regs[idx].is_dust){
                    float rel=(d_regs[idx].score-sc_base)/sc_rng;
                    if(rel<0)rel=0;if(rel>1)rel=1;
                    uint8_t fr=0,fg=0,fb=0; d_dustcol(rel,&fr,&fg,&fb);
                    d_blend(&d_outr[oy][ox],&d_outg[oy][ox],&d_outb[oy][ox],fr,fg,fb,0.72f);
                }
            }
            if(d_ero[y][x]&&d_is_boundary(y,x)){
                int idx=(int)d_lbl[y][x]-1;
                if(idx>=0&&idx<d_nreg&&d_regs[idx].is_dust){
                    d_outr[oy][ox]=0;d_outg[oy][ox]=210;d_outb[oy][ox]=230;
                }
            }
        }
    }
    d_fill(D_ROW_BODY,W,D_ROW_BODY+18,D_OUT_W-1,8,20,14);
    d_hline(D_ROW_BODY+18,W,D_OUT_W-1,0,180,80);
    d_dstr_c(D_ROW_BODY+5,W,D_OUT_W,"DUST CANDIDATE DETECTION MAP",1,100,255,160,8,20,14);
    int label_n=0;
    for(int i=0;i<d_nreg&&label_n<D_MAX_LABELS;i++){
        if(!d_regs[i].is_dust) continue;
        float rel=(d_regs[i].score-sc_base)/sc_rng;
        if(rel<0)rel=0;if(rel>1)rel=1;
        uint8_t lr,lg,lb; d_dustcol(rel,&lr,&lg,&lb);
        int px=d_regs[i].cx, py=d_regs[i].cy-20;
        if(px<4)px=4; if(px>W-40)px=W-40;
        if(py<22)py=22; if(py>H-14)py=H-14;
        char buf[8]; snprintf(buf,8,"D%d",label_n+1<100?label_n+1:99);
        int sc=2, bw2=(FW+1)*sc*3+6, bh2=FH*sc+4;
        int ry=D_ROW_BODY+py;
        d_fill(ry-2,W+px-2,ry-2+bh2,W+px-2+bw2,8,10,18);
        for(int kx=W+px-2;kx<=W+px-2+bw2;kx++){d_dp(ry-2,kx,lr,lg,lb);d_dp(ry-2+bh2,kx,lr,lg,lb);}
        for(int ky=ry-2;ky<=ry-2+bh2;ky++){d_dp(ky,W+px-2,lr,lg,lb);d_dp(ky,W+px-2+bw2,lr,lg,lb);}
        d_dstr(ry,W+px,buf,sc,lr,lg,lb,8,10,18);
        label_n++;
    }

    for(int y=D_ROW_BODY;y<D_ROW_BODY+H;y++){d_dp(y,W-1,220,220,90);d_dp(y,W,220,220,90);}

    d_fill(D_ROW_INFO,0,D_ROW_INFO+D_INFO_H-1,D_OUT_W-1,12,16,30);
    d_hline(D_ROW_INFO,0,D_OUT_W-1,0,100,200);
    d_hline(D_ROW_INFO+D_INFO_H-1,0,D_OUT_W-1,0,180,80);
    int total_dust_px=0;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        if(d_ero[y][x]&&d_lbl[y][x]){
            int idx=(int)d_lbl[y][x]-1;
            if(idx>=0&&idx<d_nreg&&d_regs[idx].is_dust) total_dust_px++;
        }
    }
    float cov=100.0f*total_dust_px/(W*H);
    const char *sev=(d_dust_count>=D_SEV_HIGH)?"HIGH":
                    (d_dust_count>=D_SEV_MED)?"MEDIUM":"LOW";
    uint8_t sc_r,sc_g,sc_b;
    if(d_dust_count>=D_SEV_HIGH){sc_r=230;sc_g=50;sc_b=50;}
    else if(d_dust_count>=D_SEV_MED){sc_r=240;sc_g=160;sc_b=0;}
    else{sc_r=60;sc_g=210;sc_b=80;}
    char buf[96];
    int row1=D_ROW_INFO+6;
    snprintf(buf,sizeof(buf),"REGIONS: %d",d_dust_count);
    d_dstr(row1,10,buf,1,160,190,240,12,16,30);
    snprintf(buf,sizeof(buf),"COVERAGE: %.2f%%",cov);
    d_dstr(row1,110,buf,1,160,190,240,12,16,30);
    snprintf(buf,sizeof(buf),"MEAN DENSITY: %.3f",d_dmean);
    d_dstr(row1,240,buf,1,110,120,145,12,16,30);
    int sx=d_dstr(row1,430,"SEVERITY:",1,130,140,160,12,16,30);
    d_dstr(row1,sx+4,sev,1,sc_r,sc_g,sc_b,12,16,30);
    d_dstr(row1+14,10,"LEFT: EDGE DENSITY HEATMAP FROM SOBEL OUTPUT",1,100,180,255,12,16,30);
    d_dstr(row1+14,350,"RIGHT: DUST CANDIDATE DETECTION MAP",1,100,255,160,12,16,30);

    d_fill(D_ROW_LEG,0,D_OUT_H-1,D_OUT_W-1,8,10,20);
    d_hline(D_ROW_LEG,0,D_OUT_W-1,40,50,80);
    int oy=D_ROW_LEG+6;
    d_dstr(oy,8,"HEATMAP SCALE:",1,120,150,200,8,10,20);
    int bx=8,bw2=220,bary=oy+10,barh=14;
    for(int x=0;x<bw2;x++){
        float t=(float)x/bw2; uint8_t hr=0,hg=0,hb=0; d_heat(t,&hr,&hg,&hb);
        for(int y=bary;y<bary+barh;y++) d_dp(y,bx+x,hr,hg,hb);
    }
    for(int x=bx-1;x<=bx+bw2;x++){d_dp(bary-1,x,80,80,80);d_dp(bary+barh,x,80,80,80);}
    for(int y=bary;y<bary+barh;y++){d_dp(y,bx-1,80,80,80);d_dp(y,bx+bw2,80,80,80);}
    for(int t=0;t<=4;t++){int tx=bx+(t*bw2)/4;for(int y=bary-2;y<=bary+barh+1;y++)d_dp(y,tx,200,200,200);}
    d_dstr(bary+barh+3,bx,"LOW",1,80,130,200,8,10,20);
    d_dstr(bary+barh+3,bx+bw2/2-6,"MEDIUM",1,200,200,80,8,10,20);
    d_dstr(bary+barh+3,bx+bw2-14,"HIGH",1,220,60,60,8,10,20);
    typedef struct{uint8_t r,g,b;const char*lbl;const char*desc;}Sw;
    Sw sw[]={
        {80,230,70,"GREEN","Light dust candidate"},
        {255,160,0,"AMBER","Moderate dust candidate"},
        {230,40,40,"RED  ","Heavy dust candidate"},
        {0,210,230,"CYAN ","Detected region boundary"},
        {255,255,100,"YELL ","Peak hotspot crosshair"},
    };
    int lx2[2]={380,680};
    int sy2=bary+barh+18;
    d_dstr(sy2-10,380,"DUST FILL KEY:",1,120,150,200,8,10,20);
    for(int i=0;i<5;i++){
        int col=(i>=3)?1:0, row2=i-(col*3);
        int sx2=lx2[col], sy3=sy2+row2*14;
        d_fill(sy3,sx2,sy3+9,sx2+11,sw[i].r,sw[i].g,sw[i].b);
        for(int x=sx2-1;x<=sx2+12;x++){d_dp(sy3-1,x,55,55,55);d_dp(sy3+10,x,55,55,55);}
        for(int y=sy3-1;y<=sy3+10;y++){d_dp(y,sx2-1,55,55,55);d_dp(y,sx2+12,55,55,55);}
        int nx=d_dstr(sy3+1,sx2+14,sw[i].lbl,1,sw[i].r,sw[i].g,sw[i].b,8,10,20);
        d_dstr(sy3+1,nx+4,sw[i].desc,1,130,135,150,8,10,20);
    }
}

/* Crack detection pipeline */
static void c_dp(int y,int x,uint8_t r,uint8_t g,uint8_t b){
    if(y<0||y>=H||x<0||x>=W)return;
    c_outr[y][x]=r;c_outg[y][x]=g;c_outb[y][x]=b;
}
static void c_fill(int y0,int x0,int y1,int x1,uint8_t r,uint8_t g,uint8_t b){
    for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++) c_dp(y,x,r,g,b);
}
static void c_rect(int y0,int x0,int y1,int x1,int t,uint8_t r,uint8_t g,uint8_t b){
    for(int k=0;k<t;k++){
        for(int x=x0;x<=x1;x++){c_dp(y0+k,x,r,g,b);c_dp(y1-k,x,r,g,b);}
        for(int y=y0;y<=y1;y++){c_dp(y,x0+k,r,g,b);c_dp(y,x1-k,r,g,b);}
    }
}

static const uint8_t CFONT[][5]={
{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
{0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
{0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
{0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x30,0x30,0x00,0x00},
{0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
{0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
{0x00,0x56,0x36,0x00,0x00},{0x00,0x08,0x14,0x22,0x41},{0x14,0x14,0x14,0x14,0x14},
{0x41,0x22,0x14,0x08,0x00},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
{0x3E,0x41,0x41,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
{0x7F,0x02,0x04,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
{0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
{0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
{0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
{0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x08,0x14,0x54,0x54,0x3C},
{0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
{0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
{0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
{0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
{0x44,0x64,0x54,0x4C,0x44}};

static void c_dc(int y,int x,char c,uint8_t r,uint8_t g,uint8_t b){
    if(c<' '||c>'z')return;
    const uint8_t *col=CFONT[(unsigned char)(c-' ')];
    for(int cx=0;cx<5;cx++){uint8_t bits=col[cx];
        for(int cy=0;cy<7;cy++) if(bits&(1<<cy)) c_dp(y+cy,x+cx,r,g,b);}
}
static void c_dt(int y,int x,const char *s,uint8_t r,uint8_t g,uint8_t b){
    for(;*s;s++,x+=6) c_dc(y,x,*s,r,g,b);
}
static void c_fswatch(int y,int x,uint8_t r,uint8_t g,uint8_t b){
    for(int dy=0;dy<8;dy++) for(int dx=0;dx<12;dx++) c_dp(y+dy,x+dx,r,g,b);
}

static void c_adaptive_threshold(void){
    int half=C_ADAPT_BLOCK/2;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        float sum=0; int n=0;
        for(int j=-half;j<=half;j++) for(int i=-half;i<=half;i++){
            int ny=y+j,nx=x+i;
            if(ny>=0&&ny<H&&nx>=0&&nx<W){sum+=raw[ny][nx];n++;}
        }
        c_abin[y][x]=(raw[y][x]>(sum/n+C_ADAPT_C))?1:0;
    }
}
static void c_remove_noise(void){
    for(int y=1;y<H-1;y++) for(int x=1;x<W-1;x++){
        if(!c_abin[y][x]) continue;
        int s=0;
        for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++) s+=c_abin[y+j][x+i];
        if(s<C_NOISE_NBR) c_abin[y][x]=0;
    }
}
static int c_zs_pass(uint8_t im[H][W],int pass){
    static uint8_t del[H][W]; int n=0; memset(del,0,sizeof(del));
    for(int y=1;y<H-1;y++) for(int x=1;x<W-1;x++){
        if(!im[y][x]) continue;
        int p2=im[y-1][x],p3=im[y-1][x+1],p4=im[y][x+1],p5=im[y+1][x+1];
        int p6=im[y+1][x],p7=im[y+1][x-1],p8=im[y][x-1],p9=im[y-1][x-1];
        int A=(!p2&&p3)+(!p3&&p4)+(!p4&&p5)+(!p5&&p6)+
              (!p6&&p7)+(!p7&&p8)+(!p8&&p9)+(!p9&&p2);
        int B=p2+p3+p4+p5+p6+p7+p8+p9;
        if(B<2||B>6||A!=1) continue;
        if(pass==0){if(p2&&p4&&p6)continue;if(p4&&p6&&p8)continue;}
        else       {if(p2&&p4&&p8)continue;if(p2&&p6&&p8)continue;}
        del[y][x]=1; n++;
    }
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) if(del[y][x]) im[y][x]=0;
    return n;
}
static void c_zhang_suen(void){
    memcpy(c_thin,c_abin,sizeof(c_abin));
    int it=0;
    while(1){int c=c_zs_pass(c_thin,0)+c_zs_pass(c_thin,1);if(!c||++it>150)break;}
}
static void c_find_impact(void){
    int half=15; float best=0;
    c_seed_cy=H/2; c_seed_cx=W/2;
    for(int y=C_BORDER+half;y<H-C_BORDER-half;y+=2)
        for(int x=C_BORDER+half;x<W-C_BORDER-half;x+=2){
            float s=0;
            for(int j=-half;j<=half;j++)
                for(int i=-half;i<=half;i++) s+=raw[y+j][x+i];
            if(s>best){best=s;c_seed_cy=y;c_seed_cx=x;}
        }
}
static void c_build_roi_label(void){
    memset(c_roi,0,sizeof(c_roi));
    for(int y=C_BORDER;y<H-C_BORDER;y++) for(int x=C_BORDER;x<W-C_BORDER;x++){
        if(!c_thin[y][x]) continue;
        int dy=y-c_seed_cy,dx=x-c_seed_cx;
        if((float)(dy*dy+dx*dx)<=(float)(C_MAX_DIST*C_MAX_DIST)) c_roi[y][x]=1;
    }
    memset(c_lbl,0,sizeof(c_lbl)); c_nreg=0; int cur=1;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        if(!c_roi[y][x]||c_lbl[y][x]) continue;
        if(c_nreg>=C_MAX_REG) continue;
        CReg *r=&c_regs[c_nreg];
        r->id=cur;r->area=0;r->x0=r->y0=9999;r->x1=r->y1=0;
        int sp=0;c_stk_x[sp]=x;c_stk_y[sp++]=y;c_lbl[y][x]=(uint16_t)cur;
        while(sp>0){
            int cx=c_stk_x[--sp],cy=c_stk_y[sp]; r->area++;
            if(cx<r->x0)r->x0=cx;if(cx>r->x1)r->x1=cx;
            if(cy<r->y0)r->y0=cy;if(cy>r->y1)r->y1=cy;
            for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++){
                int nx=cx+i,ny=cy+j;
                if(nx>=0&&ny>=0&&nx<W&&ny<H&&c_roi[ny][nx]&&!c_lbl[ny][nx]){
                    c_lbl[ny][nx]=(uint16_t)cur;c_stk_x[sp]=nx;c_stk_y[sp++]=ny;
                }
            }
        }
        c_nreg++;cur++;
    }
}
static void c_grow_crack(void){
    memset(c_crack,0,sizeof(c_crack));
    memset(c_in_crack,0,sizeof(c_in_crack));
    for(int i=0;i<c_nreg;i++){
        if(c_regs[i].area<C_MIN_FRAG) continue;
        int found=0;
        for(int y=c_regs[i].y0;y<=c_regs[i].y1&&!found;y++)
            for(int x=c_regs[i].x0;x<=c_regs[i].x1&&!found;x++){
                if(c_lbl[y][x]!=(uint16_t)c_regs[i].id) continue;
                int dy=y-c_seed_cy,dx=x-c_seed_cx;
                if((float)(dy*dy+dx*dx)<=(float)(C_SEED_RADIUS*C_SEED_RADIUS)){c_in_crack[i]=1;found=1;}
            }
        if(c_in_crack[i])
            for(int y=c_regs[i].y0;y<=c_regs[i].y1;y++)
                for(int x=c_regs[i].x0;x<=c_regs[i].x1;x++)
                    if(c_lbl[y][x]==(uint16_t)c_regs[i].id) c_crack[y][x]=1;
    }
    static uint8_t dil[H][W];
    for(int it=0;it<C_GROW_ITERS;it++){
        memcpy(dil,c_crack,sizeof(c_crack));
        for(int p=0;p<C_BRIDGE;p++){
            static uint8_t tmp[H][W]; memcpy(tmp,dil,sizeof(dil));
            for(int y=1;y<H-1;y++) for(int x=1;x<W-1;x++)
                if(tmp[y][x]||tmp[y-1][x]||tmp[y+1][x]||tmp[y][x-1]||tmp[y][x+1])
                    dil[y][x]=1;
        }
        int added=0;
        for(int i=0;i<c_nreg;i++){
            if(c_in_crack[i]||c_regs[i].area<C_MIN_FRAG) continue;
            int touch=0;
            for(int y=c_regs[i].y0;y<=c_regs[i].y1&&!touch;y++)
                for(int x=c_regs[i].x0;x<=c_regs[i].x1&&!touch;x++)
                    if(c_lbl[y][x]==(uint16_t)c_regs[i].id&&dil[y][x]) touch=1;
            if(touch){
                c_in_crack[i]=1;
                for(int y=c_regs[i].y0;y<=c_regs[i].y1;y++)
                    for(int x=c_regs[i].x0;x<=c_regs[i].x1;x++)
                        if(c_lbl[y][x]==(uint16_t)c_regs[i].id) c_crack[y][x]=1;
                added++;
            }
        }
        if(!added) break;
    }
}
static void c_dilate(void){
    memcpy(c_thick,c_crack,sizeof(c_crack));
    for(int p=0;p<C_DILATE_PX;p++){
        static uint8_t tmp[H][W]; memcpy(tmp,c_thick,sizeof(c_thick));
        for(int y=1;y<H-1;y++) for(int x=1;x<W-1;x++)
            if(tmp[y][x]||tmp[y-1][x]||tmp[y+1][x]||tmp[y][x-1]||tmp[y][x+1])
                c_thick[y][x]=1;
    }
    c_crack_area=0;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) c_crack_area+=c_thick[y][x];
}

static void c_render(const char *fname){
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        uint8_t g=(uint8_t)(raw[y][x]*C_BG_BRIGHT);
        c_outr[y][x]=c_outg[y][x]=c_outb[y][x]=g;
    }
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        if(!c_thick[y][x]) continue;
        c_outr[y][x]=255;c_outg[y][x]=230;c_outb[y][x]=0;
    }
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        if(!c_thick[y][x]) continue;
        int dy=y-c_seed_cy,dx=x-c_seed_cx;
        if((float)(dy*dy+dx*dx)<=(float)(C_IMPACT_RING*C_IMPACT_RING)){
            c_outr[y][x]=255;c_outg[y][x]=60;c_outb[y][x]=60;
        }
    }
    {
        int y0=9999,y1=0,x0=9999,x1=0;
        for(int y=0;y<H;y++) for(int x=0;x<W;x++){
            if(!c_thick[y][x])continue;
            if(y<y0)y0=y;if(y>y1)y1=y;if(x<x0)x0=x;if(x>x1)x1=x;
        }
        int pad=8;
        c_bbox_y0=y0-pad<0?0:y0-pad; c_bbox_y1=y1+pad>H-1?H-1:y1+pad;
        c_bbox_x0=x0-pad<0?0:x0-pad; c_bbox_x1=x1+pad>W-1?W-1:x1+pad;
        c_rect(c_bbox_y0,c_bbox_x0,c_bbox_y1,c_bbox_x1,3,255,50,50);
        int lx=c_bbox_x0, ly=c_bbox_y0-20; if(ly<0)ly=c_bbox_y0+4;
        c_fill(ly-2,lx,ly+9,lx+18*6+4,255,50,50);
        c_dt(ly,lx+2,"CRACK DETECTED",255,255,255);
    }
    {
        const char *sev=(c_crack_area>=C_SEV_HIGH)?"HIGH":
                        (c_crack_area>=C_SEV_MED)?"MEDIUM":"LOW";
        uint8_t scr=255,scg=200,scb=0;
        if(c_crack_area>=C_SEV_HIGH){scr=255;scg=50;scb=50;}
        c_fill(428,2,510,252,0,0,0);
        c_rect(428,2,510,252,1,80,80,80);
        c_dt(433,6,"SOLAR PANEL DEFECT REPORT",255,220,0);
        for(int x=4;x<252;x++) c_dp(446,x,80,80,80);
        c_dt(450,6,"Type    : Impact Crack",200,200,200);
        c_dt(463,6,"Severity: ",200,200,200);
        c_dt(463,66,sev,scr,scg,scb);
        char buf[64];
        sprintf(buf,"Area    : %d px",c_crack_area); c_dt(476,6,buf,200,200,200);
        sprintf(buf,"Centre  : (%d,%d)",c_seed_cx,c_seed_cy); c_dt(489,6,buf,200,200,200);
        sprintf(buf,"BBox    : (%d,%d)-(%d,%d)",c_bbox_x0,c_bbox_y0,c_bbox_x1,c_bbox_y1);
        c_dt(502,6,buf,200,200,200);
        c_dt(502,260,fname,160,160,160);
    }
    {
        int lx=360,ly=4;
        c_fill(ly,lx,ly+54,lx+150,0,0,0);
        c_rect(ly,lx,ly+54,lx+150,1,80,80,80);
        c_fswatch(ly+4, lx+4,255,230,0);  c_dt(ly+5, lx+18,"Crack edges",  200,200,200);
        c_fswatch(ly+17,lx+4,255,60,60);  c_dt(ly+18,lx+18,"Impact centre",200,200,200);
        c_fswatch(ly+30,lx+4,255,50,50);  c_dt(ly+31,lx+18,"Bounding box", 200,200,200);
        c_fswatch(ly+43,lx+4,255,255,255);c_dt(ly+44,lx+18,"Background",   200,200,200);
    }
}

/* Process a single image */
static Verdict process_one(const char *in_path,
                            const char *out_path,
                            const char *nm)
{
    memset(raw,0,sizeof(raw));
    if(!read_bmp(in_path)) return (Verdict)-1;

    Verdict v = compute_verdict();

    /* Print features for threshold tuning */
    printf("  [diag] mean=%.2f  std=%.2f  blobs=%-5d\n",
           (float)v_mean, v_pixel_std, v_total_blobs);

    if(v == V_CLEAN){
        cl_render(nm);
        write_bmp24_clean(out_path);

    } else if(v == V_DUST){
        memset(d_bin,0,sizeof(d_bin)); memset(d_grid,0,sizeof(d_grid));
        memset(d_tex,0,sizeof(d_tex)); memset(d_dens,0,sizeof(d_dens));
        memset(d_mask,0,sizeof(d_mask)); memset(d_dil,0,sizeof(d_dil));
        memset(d_ero,0,sizeof(d_ero)); memset(d_lbl,0,sizeof(d_lbl));
        memset(d_integ,0,sizeof(d_integ));
        d_nreg=0; d_dust_count=0; d_total_score=0;
        d_dmin=0; d_dmax=0; d_dmean=0; d_dstd=0;

        d_build_binary();
        d_suppress_grid();
        d_build_density();
        d_morph();
        d_cc_label();
        d_classify();
        d_render(nm);
        write_bmp24_dust(out_path);

    } else { /* CRACK branch */
        memset(c_abin,0,sizeof(c_abin)); memset(c_thin,0,sizeof(c_thin));
        memset(c_roi,0,sizeof(c_roi));   memset(c_lbl,0,sizeof(c_lbl));
        memset(c_crack,0,sizeof(c_crack));memset(c_thick,0,sizeof(c_thick));
        c_nreg=0; c_crack_area=0;
        c_seed_cy=H/2; c_seed_cx=W/2;
        c_bbox_x0=c_bbox_x1=c_bbox_y0=c_bbox_y1=0;

        c_adaptive_threshold();
        c_remove_noise();
        c_zhang_suen();
        c_find_impact();
        c_build_roi_label();
        c_grow_crack();
        c_dilate();
        c_render(nm);
        write_bmp24_crack(out_path);
    }

    return v;
}

/* Main entry */
int main(int argc, char *argv[]){
    mkdir(OUTPUT_DIR,0755);

    printf("\n╔══════════════════════════════════════════════════════╗\n");
    printf("║  Solar Panel Fault Detector  v3                      ║\n");
    printf("║  FIX: Added pixel_mean feature to verdict classifier ║\n");
    printf("║  CLEAN → greyscale copy with health banner (512×512) ║\n");
    printf("║  DUST  → dust_detect_v3 dashboard         (1024×684) ║\n");
    printf("║  CRACK → feat_multi dashboard              (512×512) ║\n");
    printf("╠══════════════════════════════════════════════════════╣\n");
    printf("║  Thresholds (edit at top of file to tune):           ║\n");
    printf("║   CLEAN : std < %.1f  AND  blobs < %d              ║\n",
           VERDICT_CLEAN_STD_MAX, VERDICT_CLEAN_BLOB_MAX);
    printf("║   CRACK : mean >= %.1f  OR  std >= %.1f          ║\n",
           VERDICT_MEAN_CRACK_MIN, VERDICT_STD_THRESH);
    printf("║   DUST  : everything else (low mean, low std)        ║\n");
    printf("║   Blob abs floor = %.0f                               ║\n",
           VERDICT_ABS_FLOOR);
    printf("╚══════════════════════════════════════════════════════╝\n\n");

    if(argc>=2){
        const char *base=argv[1];
        const char *p=strrchr(base,'/'); if(p) base=p+1;
        char out[512];
        snprintf(out,sizeof(out),"%s/%s",OUTPUT_DIR,base);
        Verdict v=process_one(argv[1],out,base);
        if((int)v>=0){
            const char *vname = (v==V_CLEAN)?"CLEAN":(v==V_DUST)?"DUST":"CRACK";
            printf("  File    : %s\n",argv[1]);
            printf("  Verdict : %s\n",vname);
            printf("  mean=%.2f  std=%.2f  blobs=%d\n",
                   (float)v_mean, v_pixel_std, v_total_blobs);
            printf("  Output  : %s\n",out);
        }
        return 0;
    }

    printf("  Input  : %s/\n  Output : %s/\n\n",INPUT_DIR,OUTPUT_DIR);
    printf("%-5s  %-38s  %-6s  %6s  %6s  %6s\n",
           "No.","Filename","Verdict","mean","std","blobs");
    printf("─────  ──────────────────────────────────────  ──────  "
           "──────  ──────  ──────\n");

    DIR *dir=opendir(INPUT_DIR);
    if(!dir){perror("ERROR");return 1;}

    int count=0, ok=0, n_clean=0, n_dust=0, n_crack=0;
    struct dirent *e;
    while((e=readdir(dir))!=NULL && count<MAX_IMAGES){
        char *nm=e->d_name; int len=(int)strlen(nm);
        if(len<5) continue;
        char *ext=nm+len-4;
        if(!(ext[0]=='.'&&(ext[1]=='b'||ext[1]=='B')&&
             (ext[2]=='m'||ext[2]=='M')&&(ext[3]=='p'||ext[3]=='P'))) continue;
        count++;
        char ip[512],op[512];
        snprintf(ip,sizeof(ip),"%s/%s",INPUT_DIR,nm);
        snprintf(op,sizeof(op),"%s/%s",OUTPUT_DIR,nm);
        Verdict v=process_one(ip,op,nm);
        if((int)v>=0){
            const char *vname=(v==V_CLEAN)?"CLEAN":(v==V_DUST)?"DUST":"CRACK";
            printf("%-5d  %-38s  %-6s  %6.1f  %6.1f  %5d\n",
                   count,nm,vname,(float)v_mean,v_pixel_std,v_total_blobs);
            if(v==V_CLEAN) n_clean++;
            else if(v==V_DUST)  n_dust++;
            else                n_crack++;
            ok++;
        } else {
            printf("%-5d  %-38s  SKIPPED\n",count,nm);
        }
    }
    closedir(dir);

    printf("\n╔══════════════════════════════════════╗\n");
    printf("║  BATCH SUMMARY                       ║\n");
    printf("╠══════════════════════════════════════╣\n");
    printf("║  Processed : %-4d                   ║\n",ok);
    printf("║  CLEAN     : %-4d                   ║\n",n_clean);
    printf("║  DUST      : %-4d                   ║\n",n_dust);
    printf("║  CRACK     : %-4d                   ║\n",n_crack);
    printf("╚══════════════════════════════════════╝\n\n");

    printf("  Tuning tip: run with a single known image and read the\n");
    printf("  [diag] line: mean=XX.XX  std=XX.XX  blobs=YYYY\n");
    printf("  Adjust VERDICT_MEAN_CRACK_MIN (currently %.1f) if\n",
           VERDICT_MEAN_CRACK_MIN);
    printf("  crack images in your dataset have different mean values.\n\n");

    return 0;
}
