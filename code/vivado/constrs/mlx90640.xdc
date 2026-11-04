#==========================================================================
# mlx90640.xdc  —�?  MLX90640 �? I2C 引脚约束
#
# 明确接口代号（本次用 J24/J25 排针上的 LCD RGB888 空闲引脚，匹�?"母�?�母"杜邦线）�?
#   SDA  �?  T10   �?= LCD_R0，J25 排针上丝印标 "R0"，BANK34�?3.3V�?7010 �?甡��?
#   SCL  �?  T12   �?= LCD_R1，J25 排针上丝印标 "R1"，BANK34�?3.3V�?7010 �?甡��?
#
# 为什么不用原来的 K16/N15：那两根圡� J9 摄像头座（排母），母对母杜邦线插不进去�?
# 若你�?"�?对母"杜邦线，仍可改回 K16(SCL)/N15(SDA)——两�?方�?�二选一，别混用�?
#
# 注意：这两个引脚�?"双向 inout"（�?�应 i2c_emio 模块�? iic_sda / iic_scl �?口）�?
# BANK34 供电 3.3V，与 MLX90640 �? 3.3V I2C 电平一致，无需电平�?�?�?
#==========================================================================

set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 PULLUP TRUE } [get_ports iic_sda_0]
set_property -dict { PACKAGE_PIN T12 IOSTANDARD LVCMOS33 PULLUP TRUE } [get_ports iic_scl_0]

# ---- 蜂鸣噡� BEEP（无源压电，U15�??----
set_property -dict { PACKAGE_PIN U14  IOSTANDARD LVCMOS33 } [get_ports beep]

