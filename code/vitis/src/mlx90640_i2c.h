/*
 * mlx90640_i2c.h  ——  MLX90640 的 I2C 底层（Zynq PS I2C0 + XIic 驱动）
 *
 * 这一层是唯一需要适配野火 7010 板的文件：
 *   MLX90640_I2CRead  /  MLX90640_I2CWrite 底层用 Xilinx 的 XIic 驱动实现，
 *   上层 mlx90640_api.c（官方补偿算法）原封不动地调用这两个函数。
 */
#ifndef _MLX90640_I2C_H_
#define _MLX90640_I2C_H_

#include <stdint.h>

/* MLX90640 的 7-bit I2C 地址（写=0x66，读=0x67）。
 * Waveshare 树莓派版默认是 0x33；个别克隆板可能是 0x77，上板先扫描确认。 */
#define MLX90640_ADDR   0x51   /* 2 字节命令扫描确认：本模块地址为 0x51 */

/* 上电初始化 PS I2C0 */
void MLX90640_I2CInit(void);

/* 从 startAddress 开始连续读 nMemAddressRead 个 16-bit 字，存到 data（小端）。
 * 返回 0 = 成功，非 0 = 失败 */
int  MLX90640_I2CRead (uint8_t slaveAddr, uint16_t startAddress,
                       uint16_t nMemAddressRead, uint16_t *data);

/* 向 writeAddress 写一个 16-bit 字。返回 0 = 成功，非 0 = 失败 */
int  MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);

#endif /* _MLX90640_I2C_H_ */
