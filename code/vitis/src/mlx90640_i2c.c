/*
 * mlx90640_i2c.c  ——  MLX90640 的 I2C 底层实现（PS I2C0 + EMIO）
 *
 * 【引脚】SDA = T10 (LCD_R1)，SCL = T12 (LCD_R2)
 *   这两根同时是 4.3 寸屏 RGB888 的红色最低两位。屏幕按 RGB565 驱动时
 *   只用 R[7:3]，R2/R1 不参与显示，因此可以与 I2C 复用：
 *   屏幕上红色最低位会跟着 I2C 数据轻微抖动（约 3%），肉眼不可见。
 *
 * 【速率】20kHz。100kHz 时长传输（1664 字节）会出错，接 4.7kΩ 上拉后可提速。
 */

#include "mlx90640_i2c.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xiicps.h"

XIicPs IicPs;   /* 去掉 static，便于 main.c 里做总线扫描 */

/* 等 I2C 总线真正空闲（STOP 发送完毕） */
static void i2c_wait_idle(void)
{
    u32 base = IicPs.Config.BaseAddress;
    volatile int n;
    for (n = 0; n < 200000; n++) {
        if ((XIicPs_ReadReg(base, XIICPS_SR_OFFSET) & XIICPS_SR_BA_MASK) == 0) {
            break;
        }
    }
    for (n = 0; n < 5000; n++) {
        __asm__ __volatile__("nop");
    }
}

void MLX90640_I2CInit(void)
{
    XIicPs_Config *cfg = XIicPs_LookupConfig(XPAR_XIICPS_0_BASEADDR);
    XIicPs_CfgInitialize(&IicPs, cfg, cfg->BaseAddress);
    XIicPs_SetSClk(&IicPs, 20000);   /* 20kHz：长传输下比 100kHz 稳 */

    /* 【关键】打开传输超时（最大 0xFF 个 SCL 周期）。
     *
     * MLX90640 在访问帧数据区时会让从机拉住 SCL 做时钟延展（clock stretching）。
     * Zynq I2C 控制器的传输超时默认是关闭的，一旦延展时间超出预期，
     * 控制器就会无限等待，而 XIicPs 的轮询接收循环内部同样没有超时 ——
     * 程序于是永久卡死在 XIicPs_MasterRecvPolled 里。
     * 打开超时后，异常情况会返回错误码，程序可以丢弃该帧继续工作。 */
    XIicPsSetTimeOut(&IicPs, 0xFF);

}



int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *data)
{
    uint8_t  cmd[2];
    uint8_t *bp = (uint8_t *)data;
    int      status;

    cmd[0] = (uint8_t)(startAddress >> 8);
    cmd[1] = (uint8_t)(startAddress & 0xFF);

    /* 复合读：写 2 字节地址（不发 STOP）→ 紧随重复起始接收。
     * 注意：本平台上这个写法能读出数据，但地址是否真正生效需要用不同地址
     * 分别读取来验证（见 main.c 的 FRAME-PROBE 诊断）。 */
    /* 【重复起始】MLX90640 要求"写地址 + 重复起始 + 读数据"的复合时序，
     * 中间不能有 STOP。 */
    XIicPs_SetOptions(&IicPs, XIICPS_REP_START_OPTION);
    status = XIicPs_MasterSendPolled(&IicPs, cmd, 2, slaveAddr);
    if (status != XST_SUCCESS) {
        XIicPs_SetOptions(&IicPs, 0);
        return -1;
    }

    status = XIicPs_MasterRecvPolled(&IicPs, bp, (int)(nMemAddressRead * 2), slaveAddr);
    if (status != XST_SUCCESS) {
        XIicPs_SetOptions(&IicPs, 0);
        return -1;
    }
    XIicPs_SetOptions(&IicPs, 0);

    for (uint16_t i = 0; i < nMemAddressRead * 2; i += 2) {
        uint8_t t = bp[i];
        bp[i]     = bp[i + 1];
        bp[i + 1] = t;
    }
    return 0;
}


int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)(writeAddress >> 8);
    buf[1] = (uint8_t)(writeAddress & 0xFF);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data & 0xFF);

    XIicPs_SetOptions(&IicPs, 0);
    if (XIicPs_MasterSendPolled(&IicPs, buf, 4, slaveAddr) != XST_SUCCESS) {
        return -1;
    }
    i2c_wait_idle();
    return 0;
}