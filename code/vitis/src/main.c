/*
 * main.c  ——  MLX90640 热斑方位监测（野火 ZYNQ-7010，PS 全干）
 *
 * 流程：I2C 读 MLX90640 原始帧 → 浮点补偿得到 32×24 温度图
 *       → 找最高温像素(热斑方位) + 热像素包围盒 → 报警分级
 *       → UART0 反馈（二进制温度图 + 一行 ASCII 摘要）
 *
 * 编译依赖（Vitis 2023.2，工程 hot_pot）：
 *   需要把 mlx90640_i2c.c / mlx90640_api.c / main.c 一起加进工程，
 *   并给链接器加 -lm（补偿算法用了 pow/sqrt/fabs）。
 */
#include "xparameters.h"
#include "xuartps.h"
#if USE_ALARM_GPIO
#include "xgpio.h"
#endif
#include "mlx90640_i2c.h"
#include "xiicps.h"
extern XIicPs IicPs;
#include "mlx90640_api.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>   /* qsort */
#include "sleep.h"     /* usleep */

/* ------------------------------ 可调参数 ------------------------------ */
#define BAUD          115200     /* 串口波特率（115200 最稳，viewer.py 用；921600 在 Zynq 上误差大） */

#define REFRESH_RATE  0x05       /* 帧率编码：0x05 = 16Hz（Waveshare FPS16HZ） */
#define TA_SHIFT      12          /* 反射温度修正：tr = Ta - 8（官方默认） （改12）*/
#define EMISSIVITY    0.95f      /* 发射率（多数物体 0.95，可调） */

#define WIDTH         32
#define HEIGHT        24
#define PIXELS        (WIDTH * HEIGHT)

/* 热斑判据：用「相对整幅温度中位数的温差」，而不是绝对温度。
 *
 * 原因：监测太阳能板时，晴天整板温度可能 50~65℃，如果用电绝对阈值 35℃，
 *      全部像素都会被判成"热斑"，报警完全失效。
 *      热斑的真实特征是"比周围电池片高 15~30℃"，所以用中位数 + 温差来判定。
 *
 * 参考值：正常发电的电池片之间温差 < 5℃；
 *        热斑区域通常比整板中位数高 10~30℃。 */
#define HOT_DELTA        10.0f   /* 高于中位数 10℃ → 算热斑像素 */
#define TEMP_ALARM_DELTA 25.0f   /* 高于中位数 25℃ → 2 级严重热斑 */
#define AREA_ALARM        6      /* 热斑像素数 >= 6 → 1 级一般报警 */

/* 旧的绝对阈值（改成相对判据后不再使用，保留备查）
 * #define TEMP_ALARM     45.0f
 * #define HOT_PIX_THRESH 35.0f
 */

/* 报警 GPIO 开关：0=不接 LED/蜂鸣器（第一次先跑通测温+传图，Vivado 里不用加 AXI GPIO）；
 * 1=接了 AXI GPIO（LED0=G20、LED1=J20、蜂鸣器=U15）。要设 1 时，先在 Vivado
 * Block Design 里加好 AXI GPIO 并把这三个脚约束到 XDC，再回来把这里改成 1。 */
#define USE_ALARM_GPIO 0

/* ------------------------------ 全局变量 ------------------------------ */
static XUartPs        uart;
#if USE_ALARM_GPIO
static XGpio          gpio;       /* 报警输出用 AXI GPIO */
#endif
static uint16_t       ee[832];    /* EEPROM 标定数据（上电读一次） */
static uint16_t       frame[834]; /* 一帧原始数据（768 像素 + 64 辅助 + 2 状态） */
static float          to[768];    /* 补偿后的温度图（℃） */
static paramsMLX90640 params;     /* 解析出的标定参数 */

/* ------------------------------ UART 底层 ------------------------------ */
/* 发一个字节（阻塞，查 TXFULL 位 —— Vitis 2023.2 没有 XUartPs_SendByte） */
static void send_byte(uint8_t b)
{
    uint32_t base = uart.Config.BaseAddress;
    while (XUartPs_ReadReg(base, XUARTPS_SR_OFFSET) & XUARTPS_SR_TXFULL) {
        ;
    }
    XUartPs_WriteReg(base, XUARTPS_FIFO_OFFSET, b);
}

static void send_bytes(const uint8_t *p, int n)
{
    while (n--) {
        send_byte(*p++);
    }
}

static void uart_puts(const char *s)
{
    while (*s) {
        send_byte((uint8_t)*s++);
    }
}

/* 打印无符号整数（十进制） */
static void put_u32(uint32_t v)
{
    char buf[12];
    int  i = 0;
    if (v == 0) {
        send_byte('0');
        return;
    }
    while (v) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    while (i) {
        send_byte(buf[--i]);
    }
}

/* 打印有符号整数 */
static void put_int(int32_t v)
{
    if (v < 0) {
        send_byte('-');
        v = -v;
    }
    put_u32((uint32_t)v);
}

/* 打印温度（1 位小数），避免 newlib-nano 不支持 %f 的问题 */
static void put_temp(float v)
{
    if (v < 0.0f) {
        send_byte('-');
        v = -v;
    }
    int32_t i = (int32_t)v;
    int32_t d = (int32_t)((v - (float)i) * 10.0f + 0.5f);
    if (d >= 10) {
        i++;
        d = 0;
    }
    put_int(i);
    send_byte('.');
    send_byte('0' + (uint8_t)d);
}

/* ------------------------------ 报警输出 ------------------------------ */
/* 用 AXI GPIO 驱动 LED/蜂鸣器（仅 USE_ALARM_GPIO=1 时编译）。位映射：
 *   bit0 = LED0(G20), bit1 = LED1(J20), bit2 = 蜂鸣器(U15) */
#if USE_ALARM_GPIO
static void alarm_init(void)
{
    XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_DEVICE_ID);
    XGpio_SetDataDirection(&gpio, 1, 0x0);   /* 通道1 全输出 */
    XGpio_DiscreteWrite(&gpio, 1, 0x0);
}

static void alarm_set(int level)
{
    static const uint32_t bits[3] = {
        0x0,        /* 0 级：全灭 */
        0x1,        /* 1 级：LED0 亮 */
        0x6,        /* 2 级：LED1 + 蜂鸣器 */
    };
    XGpio_DiscreteWrite(&gpio, 1, bits[level]);
}
#endif

/* ------------------------------ 蜂鸣器 ------------------------------ */
/* 底板蜂鸣器：BEEP(FPGA U15) -- R100 1K --> Q2(SS8050 NPN) --> PZ1 无源压电蜂鸣器
 *   - BEEP 高电平 → 三极管导通 → 蜂鸣器得电（高电平有效）
 *   - PZ1 是【无源】压电片，必须 2~4kHz 方波；方波由 PL 的 buzzer_tone 模块产生
 *
 * 控制路径：PS GPIO EMIO bit0 → PL 的 buzzer_tone.en
 *   软件只需写一个寄存器位，方波由硬件持续输出，不占 CPU（读帧阻塞时也照响）。
 *
 * Zynq-7000 GPIO 寄存器（EMIO 属于 bank2）：
 *   DIRM  0xE000A284  bit0=1 → 输出
 *   OEN   0xE000A288  bit0=1 → 输出使能
 *   DATA  0xE000A010  bit0    → 输出值（EMIO[31:0] 数据低 16 位）
 */
#define GPIO_BASE       0xE000A000u
#define GPIO_DATA2_LSW  (GPIO_BASE + 0x0010u)
#define GPIO_DATA2_MSW  (GPIO_BASE + 0x0014u)
#define GPIO_DIRM2      (GPIO_BASE + 0x0284u)
#define GPIO_OEN2       (GPIO_BASE + 0x0288u)

static void buzzer_init(void)
{
    /*
     * EMIO GPIO 现在当作「并行口」用：低 16 位像素数据、16~29 位显存地址、
     * 30 位写入脉冲、31 位蜂鸣器使能。所以 32 位全部设为输出。
     */
    *(volatile uint32_t *)GPIO_DIRM2 = 0xFFFFFFFFu;
    *(volatile uint32_t *)GPIO_OEN2  = 0xFFFFFFFFu;
    *(volatile uint32_t *)GPIO_DATA2_LSW = 0u;
    *(volatile uint32_t *)GPIO_DATA2_MSW = 0u;
}

/* EMIO[31:16] 的影子寄存器：只保留蜂鸣器位，其余字段每次现算 */
static uint32_t gpo_msw = 0u;

static void buzzer_set(int on)
{
    if (on) gpo_msw |= 0x8000u;          /* bit31 = 蜂鸣器 */
    else    gpo_msw &= ~0x8000u;
    /* 写回时确保脉冲位为 0，避免误触发显存写入 */
    *(volatile uint32_t *)GPIO_DATA2_MSW = gpo_msw & ~0x4000u;
}

/* 往 PL 显存写一个像素（地址 0~16383，颜色 RGB565） */
static void fb_put(uint16_t addr, uint16_t color)
{
    volatile uint32_t *msw = (volatile uint32_t *)GPIO_DATA2_MSW;
    volatile uint32_t *lsw = (volatile uint32_t *)GPIO_DATA2_LSW;
    uint32_t base = (gpo_msw & 0x8000u) | (uint32_t)addr;

    *msw = base;                 /* 地址就位，脉冲=0 */
    *lsw = (uint32_t)color;      /* 数据就位 */
    *msw = base | 0x4000u;       /* 脉冲上升沿 → PL 锁存写入 */
    *msw = base;                 /* 脉冲回低，为下一次准备 */
}

/* ------------------------- LCD 显存与伪彩色 ------------------------- */
/* PL 侧的 AXI BRAM 分配在 0x40000000（在 Block Design 的地址编辑器里指定），
 * 这里写死，这样不需要重新生成 BSP 的 xparameters。 */
#define DEMO_MODE 0   /* 1=演示模式：不访问传感器，只显示测试画面 */
#define LCD_VRAM_BASE 0x40000000u
#define LCD_TEST_PATTERN 0   /* 1=四象限色块诊断图案，0=正常热力图 */

/* 温度 → RGB565 伪彩色：蓝 → 青 → 绿 → 黄 → 红 五段线性插值 */
static uint16_t color565(float t, float lo, float hi)
{
    float n = (t - lo) / (hi - lo);
    float r = 0.0f, g = 0.0f, b = 0.0f;

    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;

    if (n < 0.25f) {                    /* 蓝 → 青 */
        float k = n / 0.25f;
        r = 0.0f;  g = k;         b = 1.0f;
    } else if (n < 0.5f) {              /* 青 → 绿 */
        float k = (n - 0.25f) / 0.25f;
        r = 0.0f;  g = 1.0f;      b = 1.0f - k;
    } else if (n < 0.75f) {             /* 绿 → 黄 */
        float k = (n - 0.5f) / 0.25f;
        r = k;     g = 1.0f;      b = 0.0f;
    } else {                            /* 黄 → 红 */
        float k = (n - 0.75f) / 0.25f;
        r = 1.0f;  g = 1.0f - k;  b = 0.0f;
    }

    return (uint16_t)(((uint16_t)(r * 31.0f) << 11) |
                      ((uint16_t)(g * 63.0f) << 5)  |
                      ((uint16_t)(b * 31.0f)));
}

/* ------------------------------ 中位数 ------------------------------ */
/* 求整幅温度图的中位数，作为"整板基准温度"。
 * 用中位数而不是平均值：热斑只占少量像素，不会把中位数拉偏。 */
static int cmp_float_asc(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

static float calc_median(const float *src, int n)
{
    static float tmp[PIXELS];
    memcpy(tmp, src, (size_t)n * sizeof(float));
    qsort(tmp, (size_t)n, sizeof(float), cmp_float_asc);
    return tmp[n / 2];
}

/* ------------------------------ 主程序 ------------------------------ */

/* ==================== 屏幕信息区绘制 ====================
 * 信息区在热力图下方的黑带里，取中间 32 行（480×32），
 * 显存地址 = 1024 + 行*480 + 列，正好落在 16384 个字以内。
 *
 * 字体是 3×5 点阵，放大 3 倍 → 每字符 9×15 像素，再留 3 像素间距。
 * 只做了用得到的字符：0-9 . - 空格 和 A C E G I L M N V X R S T O K D P
 */
#define INFO_BASE_WORD  1024u
#define INFO_W          480
#define INFO_H          32
#define INFO_ROW_OFFSET 12          /* 在 56 行黑带里的起始行 */

static const char font_chars[] = "0123456789.- ACEGILMNVXRSTOKDP";

/* 每个字符 5 行 × 3 位（bit2 = 最左像素） */
static const unsigned char font_3x5[][5] = {
    {7,5,5,5,7},   /* 0 */
    {2,6,2,2,7},   /* 1 */
    {7,1,7,4,7},   /* 2 */
    {7,1,7,1,7},   /* 3 */
    {5,5,7,1,1},   /* 4 */
    {7,4,7,1,7},   /* 5 */
    {7,4,7,5,7},   /* 6 */
    {7,1,1,1,1},   /* 7 */
    {7,5,7,5,7},   /* 8 */
    {7,5,7,1,7},   /* 9 */
    {0,0,0,0,2},   /* . */
    {0,0,7,0,0},   /* - */
    {0,0,0,0,0},   /* 空格 */
    {2,5,7,5,5},   /* A */
    {7,4,4,4,7},   /* C */
    {7,4,7,4,7},   /* E */
    {7,4,5,5,7},   /* G */
    {7,2,2,2,7},   /* I */
    {4,4,4,4,7},   /* L */
    {5,7,7,5,5},   /* M */
    {5,7,7,7,5},   /* N */
    {5,5,5,5,2},   /* V */
    {5,5,2,5,5},   /* X */
    {6,5,6,5,5},   /* R */
    {7,4,7,1,7},   /* S */
    {7,2,2,2,2},   /* T */
    {7,5,5,5,7},   /* O */
    {5,5,6,5,5},   /* K */
    {6,5,5,5,6},   /* D */
    {7,5,7,4,4},   /* P */
};

/* 写信息区的一个像素（x: 0~479, y: 0~31） */
static void info_px(int x, int y, uint16_t color)
{
    if (x < 0 || x >= INFO_W || y < 0 || y >= INFO_H) return;
    fb_put((uint16_t)(INFO_BASE_WORD + (uint16_t)y * INFO_W + (uint16_t)x), color);
}

static void info_rect(int x0, int y0, int w, int h, uint16_t color)
{
    int x, y;
    for (y = y0; y < y0 + h; y++)
        for (x = x0; x < x0 + w; x++)
            info_px(x, y, color);
}

/* 画一个 3×5 字符，scale 为放大倍数 */
static void info_char(int x, int y, char ch, int scale, uint16_t color)
{
    int i, r, c, idx = -1;
    for (i = 0; font_chars[i]; i++) {
        if (font_chars[i] == ch) { idx = i; break; }
    }
    if (idx < 0) return;
    for (r = 0; r < 5; r++) {
        for (c = 0; c < 3; c++) {
            if (font_3x5[idx][r] & (4 >> c)) {
                info_rect(x + c * scale, y + r * scale, scale, scale, color);
            }
        }
    }
}

/* 画一串字符，返回结束时的 x */
static int info_str(int x, int y, const char *s, int scale, uint16_t color)
{
    while (*s) {
        info_char(x, y, *s, scale, color);
        x += 4 * scale;
        s++;
    }
    return x;
}

/* 把温度值（乘 10）转成 "32.5C" 形式 */
static void temp_to_str(int t10, char *buf)
{
    int p = 0;
    if (t10 < 0) { buf[p++] = '-'; t10 = -t10; }
    if (t10 >= 1000) { buf[p++] = (char)('0' + (t10 / 1000) % 10); }
    if (t10 >= 100)  { buf[p++] = (char)('0' + (t10 / 100) % 10); }
    buf[p++] = (char)('0' + (t10 / 10) % 10);
    buf[p++] = '.';
    buf[p++] = (char)('0' + t10 % 10);
    buf[p++] = 'C';
    buf[p] = 0;
}

/* 整屏刷新信息区：最高温 / 平均温 / 报警等级 */
static void draw_text_rgb565(float tmax, float tmin, float tavg, int level, int hot_cnt)
{
    char buf[12];
    int x, y;

    /* 背景 */
    info_rect(0, 0, INFO_W, INFO_H, 0x0000);

    /* 左边：报警等级指示块（绿/黄/红），越高级越亮 */
    {
        uint16_t box;
        uint16_t edge = 0x7BEF;      /* 灰白边框 */
        if (level == 0)      box = 0x07E0;   /* 绿：正常 */
        else if (level == 1) box = 0xFFE0;   /* 黄：一般热斑 */
        else                 box = 0xF800;   /* 红：严重 */
        info_rect(6, 3, 56, 26, edge);
        info_rect(8, 5, 52, 22, box);
        /* 等级数字画在色块中间（黑字） */
        buf[0] = (char)('0' + (level > 9 ? 9 : level));
        buf[1] = 0;
        info_char(8 + 20, 5 + 4, buf[0], 4, 0x0000);
    }

    /* 上面一行：最高温 */
    info_str(76, 1, "MAX", 2, 0xFFFF);
    temp_to_str((int)(tmax * 10.0f + (tmax >= 0 ? 0.5f : -0.5f)), buf);
    info_str(76 + 4 * 2 * 3 + 8, 1, buf, 2, 0xF800);

    /* 下面一行：平均温 + 热斑面积 */
    info_str(76, 17, "AVG", 2, 0xFFFF);
    temp_to_str((int)(tavg * 10.0f + (tavg >= 0 ? 0.5f : -0.5f)), buf);
    info_str(76 + 4 * 2 * 3 + 8, 17, buf, 2, 0x07FF);

    /* 右侧：热斑像素数（面积） */
    info_str(320, 17, "AREA", 2, 0xFFFF);
    {
        int p = 0, v = hot_cnt;
        char nb[6];
        if (v == 0) { nb[p++] = '0'; }
        else {
            char rv[6]; int n = 0;
            while (v > 0 && n < 6) { rv[n++] = (char)('0' + v % 10); v /= 10; }
            while (n > 0) { nb[p++] = rv[--n]; }
        }
        nb[p] = 0;
        info_str(320 + 4 * 2 * 4 + 8, 17, nb, 2, 0xFFE0);
    }

    /* 最低温放右上，凑满信息量 */
    info_str(320, 1, "MIN", 2, 0xFFFF);
    temp_to_str((int)(tmin * 10.0f + (tmin >= 0 ? 0.5f : -0.5f)), buf);
    info_str(320 + 4 * 2 * 3 + 8, 1, buf, 2, 0x07FF);
}


/* 开机 / 无数据时的占位画面：深蓝渐变 + 提示文字，避免黑屏被误认为死机 */
static void draw_standby_screen(void)
{
    int i;
    for (i = 0; i < PIXELS; i++) {
        int r = i >> 5, c = i & 31;
        int b = 30 - ((r + c) >> 2);
        if (b < 2) b = 2;
        fb_put((uint16_t)i, (uint16_t)(b & 0x1F));   /* 深蓝色阶 */
    }
    info_rect(0, 0, INFO_W, INFO_H, 0x0000);
    info_str(24, 10, "NO DATA", 3, 0xFFE0);
    info_str(230, 12, "CHECK SENSOR", 2, 0xFFFF);
}

int main(void)
{
    int status;

    /* 1. 初始化 UART0（反馈） */
    XUartPs_Config *uart_cfg = XUartPs_LookupConfig(XPAR_XUARTPS_0_BASEADDR);
    XUartPs_CfgInitialize(&uart, uart_cfg, uart_cfg->BaseAddress);
    XUartPs_SetOperMode(&uart, XUARTPS_OPER_MODE_NORMAL);
    XUartPs_SetBaudRate(&uart, BAUD);

    uart_puts("UART0 ok\r\n");








    buzzer_init();   /* 先把 GPIO 配成输出，否则下面写显存无效 */

    /* ===== STARTUP-PATTERN：开机先画一屏四象限 + 信息区 =====
     * 目的：即使传感器不应答（I2C 卡在初始化阶段），屏幕也一定有内容，
     * 可用于演示"显示通路正常"。后面若能读到真实温度帧，会被热图覆盖。 */
    {
        int i;
        for (i = 0; i < PIXELS; i++) {
            int rr = i >> 5;          /* 行 0~23 */
            int cc = i & 31;          /* 列 0~31 */
            uint16_t col;
            if (rr < 12) col = (cc < 16) ? 0xF800u : 0x07E0u;   /* 红 / 绿 */
            else         col = (cc < 16) ? 0x001Fu : 0xFFFFu;   /* 蓝 / 白 */
            fb_put((uint16_t)i, col);
        }
        draw_text_rgb565(0.0f, 0.0f, 0.0f, 0, 0);
    }



#if DEMO_MODE
    /* ===== 演示模式：不访问传感器，屏幕显示测试画面 ===== */
    uart_puts("DEMO MODE: sensor skipped, display test pattern only\r\n");
    while (1) {
        uart_puts("demo running...\r\n");
        { volatile int d; for (d = 0; d < 3000000; d++) { __asm__ __volatile__("nop"); } }
    }
#endif

      /* 调试：确认串口通了 */

    /* 2. 初始化 I2C0（读传感器） */

    /* ===== PIN-LEVEL-PROBE：读 I2C 两根线的真实电平 =====
     * PL 里把 IOBUF 的引脚电平接到了 EMIO GPIO 的 bit0(SCL) / bit1(SDA)。
     * 把这两位设成输入（DIRM=0），读只读数据寄存器即可看到线的实际状态。 */
    {
        volatile unsigned int *dirm2   = (volatile unsigned int *)0xE000A284u;
        volatile unsigned int *data2ro = (volatile unsigned int *)0xE000A060u;
        unsigned int v;
        int k;

        *dirm2 &= ~0x3u;          /* bit0/bit1 设为输入 */

        uart_puts("PIN-LEVEL-PROBE:\r\n");
        for (k = 0; k < 4; k++) {
            volatile int d;
            v = *data2ro;
            uart_puts("  SCL=");
            uart_puts((v & 0x1u) ? "1" : "0");
            uart_puts("  SDA=");
            uart_puts((v & 0x2u) ? "1" : "0");
            uart_puts("\r\n");
            for (d = 0; d < 3000000; d++) { __asm__ __volatile__("nop"); }
        }
    }

    MLX90640_I2CInit();
    /* ===== I2C-SCAN：扫描 0x08~0x77，看总线上有谁应答 =====
     * 用单字节"写"探测：写操作不会因为 NACK 而挂起（TX 完成即返回），
     * 所以这个扫描是安全的。 */
    {
        int a;
        int found = 0;
        uint8_t cmd2[2] = {0x24, 0x00};   /* 2 字节寄存器地址：MLX90640 只认完整命令 */
        uart_puts("I2C-SCAN start (2-byte cmd)\r\n");
        for (a = 0x08; a < 0x78; a++) {
            if (XIicPs_MasterSendPolled(&IicPs, cmd2, 2, (u16)a) == XST_SUCCESS) {
                uart_puts("  ACK at 0x");
                put_u32((unsigned)a);
                uart_puts("\r\n");
                found++;
            }
        }
        uart_puts("I2C-SCAN done, found=");
        put_int(found);
        uart_puts("\r\n");
    }

    /* ===== DEV-PROBE：读几个寄存器，判断 0x51 是什么设备 ===== */
    {
        static const unsigned short regs[5] = {0x0000, 0x2400, 0x240E, 0x800D, 0x0400};
        unsigned short w[2];
        int t, rc;
        uart_puts("DEV-PROBE\r\n");
        for (t = 0; t < 5; t++) {
            w[0] = 0;
            rc = MLX90640_I2CRead(0x51, regs[t], 1, w);
            uart_puts("  reg=0x"); put_u32((unsigned)regs[t]);
            uart_puts(" rc="); put_int(rc);
            uart_puts(" val="); put_u32(w[0]);
            uart_puts("\r\n");
        }
        uart_puts("DEV-PROBE done\r\n");
    }




    /* 初始化蜂鸣器控制（PS GPIO EMIO bit0 → PL 方波 → BEEP = U14） */
/* buzzer_init 已在上面调用 */

    /*
     * 把刷新率设为 0.5Hz（2 秒一帧）。
     * 原因：本板 I2C 只能用 20kHz 才不出错，读一帧（1664 字节）约需 750ms。
     * 如果传感器跑 2Hz（500ms 一帧），读完再查"新帧"标志时新帧又到了，
     * GetFrameData 会一直清不掉标志并返回 -8。
     * 0x1801 = 18 位分辨率 + 0.5Hz + 棋盘模式。
     */
    /* 不写 0x800D：出厂默认 0x1901 就是正确配置，
     * 实测写 0x1801 会清掉 bit8，可能让传感器进入异常输出模式。 */
    /* 不写 0x800D：实测写这个寄存器会让传感器停止刷新（就绪位永不置位），
     * 直接使用出厂默认值（1Hz）即可。 */

    /* 3. 初始化报警 GPIO（USE_ALARM_GPIO=0 时自动跳过，不影响测温） */
#if USE_ALARM_GPIO
    alarm_init();
    alarm_set(0);
#endif

    /* 4. 传感器初始化：读 EEPROM 标定并解析参数
     * 本板 I2C 偶有错字，ExtractParameters 会因参数前后矛盾而报错，
     * 所以重试几次，直到读出一份自洽的标定数据。
     * 注意：不调用 SetRefreshRate/SetChessMode（它们是读-改-写，会写坏 0x800D）。 */
    {
        int tries;
        status = -1;
        for (tries = 0; tries < 20; tries++) {
            if (MLX90640_DumpEE(MLX90640_ADDR, ee) != 0) {
                continue;
            }
            if (MLX90640_ExtractParameters(ee, &params) == 0) {
                status = 0;
                uart_puts("EE ok after ");
                put_int(tries + 1);
                uart_puts(" try(s)\r\n");
                break;
            }
        }
        if (status != 0) {
            uart_puts("ERR: EE/Params failed\r\n");
        }
    }

    uart_puts("MLX90640 hotspot monitor ready\r\n");
        draw_standby_screen();   /* 先画占位画面 */



    

    /* 5. 主循环 */
    while (1) {
        /* 读一帧（阻塞到新帧就绪） */
        status = MLX90640_GetFrameData(MLX90640_ADDR, frame);
        if (status < 0) {
            uart_puts("ERR: GetFrame code=");
            put_int(status);
            uart_puts("\r\n");
            continue;
        }

        /* 浮点补偿 → 32×24 温度图 */
        float ta = MLX90640_GetTa(frame, &params);
        float vdd = MLX90640_GetVdd(frame, &params);

        /* 帧完整性过滤：Ta 或 Vdd 明显不合理说明这一帧读完就坏了，丢掉 */
        if (ta < -20.0f || ta > 80.0f || vdd < 2.5f || vdd > 4.0f) {
            continue;
        }

        float tr = ta - TA_SHIFT;
        MLX90640_CalculateTo(frame, &params, EMISSIVITY, tr, to);

        /* 二次过滤：整幅温度超出合理区间也丢掉，避免画出几百度 */
        float t_frame_min = to[0], t_frame_max = to[0];
        {
            int i;
            for (i = 1; i < PIXELS; i++) {
                if (to[i] < t_frame_min) t_frame_min = to[i];
                if (to[i] > t_frame_max) t_frame_max = to[i];
            }
            if (t_frame_max > 80.0f || t_frame_min < -20.0f) {
                continue;
            }
        }

        /* ---- 把温度图渲染成 RGB565 写进 PL 的显存（0x40000000）----
         * 显存第 0~767 个字对应 32×24 个热力块，PL 负责放大到 480×216。
         * 色标范围按本帧自动拉伸（和电脑端 viewer.py 一致）。 */
        {
            float lo = t_frame_min;
            float hi = t_frame_max;
            int i;
            if (hi - lo < 2.0f) {           /* 整幅几乎同温，别让色标退化 */
                lo -= 1.0f;
                hi += 1.0f;
            }
            /* 显存：一个 32 位字放 1 个像素（只用低 16 位） */
#if LCD_TEST_PATTERN
            /* 诊断图案：4 个象限各一色，用来确认行列映射
             *   左上 红   右上 绿
             *   左下 蓝   右下 白  */
            for (i = 0; i < PIXELS; i++) {
                int rr = i >> 5;          /* 行 0~23 */
                int cc = i & 31;          /* 列 0~31 */
                uint16_t col;
                if (rr < 12) col = (cc < 16) ? 0xF800u      /* 红 */
                                             : 0x07E0u;     /* 绿 */
                else         col = (cc < 16) ? 0x001Fu      /* 蓝 */
                                             : 0xFFFFu;     /* 白 */
                fb_put((uint16_t)i, col);
            }
#else
            for (i = 0; i < PIXELS; i++) {
                fb_put((uint16_t)i, color565(to[i], lo, hi));
            }
#endif

        }

        /* 找最高温像素（热斑中心） + 热像素包围盒
         * 判据改为「相对整幅温度中位数」：
         *   太阳能板在阳光下整板可能 50~65℃，绝对阈值 35℃ 会把整板都算成热斑。
         *   用 T > 中位数 + HOT_DELTA 只抓局部异常点。 */
        float tmed = calc_median(to, PIXELS);
        float thot = tmed + HOT_DELTA;

        float max_t = -999.0f;
        int   max_c = 0, max_r = 0;
        int   hot_cnt = 0;
        int   min_c = WIDTH, max_cbox = 0, min_r = HEIGHT, max_rbox = 0;

        for (int r = 0; r < HEIGHT; r++) {
            for (int c = 0; c < WIDTH; c++) {
                float t = to[r * WIDTH + c];
                if (t > max_t) {
                    max_t = t;
                    max_c = c;
                    max_r = r;
                }
                if (t > thot) {
                    hot_cnt++;
                    if (c < min_c)    min_c = c;
                    if (c > max_cbox) max_cbox = c;
                    if (r < min_r)    min_r = r;
                    if (r > max_rbox) max_rbox = r;
                }
            }
        }

        /* 报警分级（同样用相对温差） */
        int level = 0;
        if (max_t >= tmed + TEMP_ALARM_DELTA) {
            level = 2;
        } else if (hot_cnt >= AREA_ALARM) {
            level = 1;
        }
#if USE_ALARM_GPIO
        alarm_set(level);
#endif
        /* ---- 把温度与报警信息画到屏幕底部信息区（每帧刷新）---- */
        {
            float tmin_f = to[0], tmax_f = to[0], tsum = 0.0f;
            int k;
            for (k = 0; k < PIXELS; k++) {
                if (to[k] < tmin_f) tmin_f = to[k];
                if (to[k] > tmax_f) tmax_f = to[k];
                tsum += to[k];
            }
            draw_text_rgb565(tmax_f, tmin_f, tsum / (float)PIXELS, level, hot_cnt);
        }


        /* 蜂鸣器（PL 硬件产生方波，这里只开关使能位）：
         *   0 级 → 静音
         *   1 级（一般热斑）→ 每帧短促"嘀"一声（120ms）
         *   2 级（严重热斑）→ 连续长鸣
         * 只在真正检测到热斑时响，平时完全静音。 */
        if (level >= 2) {
            buzzer_set(1);
        } else if (level == 1) {
            buzzer_set(1);
            usleep(120000);
            buzzer_set(0);
        } else {
            buzzer_set(0);
        }

        /* ---- 反馈 1：二进制温度图（viewer.py 解析） ----
         * [FF FF][W][H][W×H×2 字节，每像素大端 uint16 = 温度×100] */
        static const uint8_t hdr[4] = { 0xFF, 0xFF, WIDTH, HEIGHT };
        send_bytes(hdr, 4);
        for (int i = 0; i < PIXELS; i++) {
            int t100 = (int)(to[i] * 100.0f + 0.5f);
            if (t100 < 0)      t100 = 0;
            if (t100 > 65535)  t100 = 65535;
            send_byte((uint8_t)((t100 >> 8) & 0xFF));
            send_byte((uint8_t)(t100 & 0xFF));
        }

        /* ---- 反馈 2：一行 ASCII 摘要（人眼直读） ---- */
        uart_puts("hot=("); put_int(max_c); uart_puts(","); put_int(max_r); uart_puts(")");
        uart_puts(" max=");  put_temp(max_t);   uart_puts("C");
        uart_puts(" med=");  put_temp(tmed);    uart_puts("C");
        uart_puts(" area="); put_int(hot_cnt);
        /* 没有热像素时不输出 box= 字段，viewer.py 就不会画那个红框 */
        if (hot_cnt > 0) {
            uart_puts(" box=("); put_int(min_c); uart_puts(".."); put_int(max_cbox);
            uart_puts(",");      put_int(min_r); uart_puts(".."); put_int(max_rbox);
            uart_puts(")");
        }
        uart_puts(" level="); put_int(level);
        uart_puts("\r\n");
    }

    return 0;
}
