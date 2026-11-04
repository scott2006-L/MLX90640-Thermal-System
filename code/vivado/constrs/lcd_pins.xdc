#==========================================================================
# 4.3 寸 480x272 RGB 屏（野火 RGB 接口电容触摸屏）
#
# 引脚来源：底板原理图第 4 页 BTB 连接器对照表（ball 名 + IO 网络名 + 功能名）
#   红 T15/T14/V13/U13/U12 = R7/R6/R5/R4/R3，T11 = R0（未用位，驱 0）
#   T10 = R1、T12 = R2 被 I2C 占用（RGB565 用不到这两位）
#   绿 R14/P14/P16/P15/U17/T16 = G7/G6/G5/G4/G3/G2，V18/V17 = G1/G0（驱 0）
#   蓝 R17/R16/U19/U18/W19 = B7/B6/B5/B4/B3，W18/W16/V16 = B2/B1/B0（驱 0）
#   同步 W15=PCLK  V12=HSYNC  W13=VSYNC  V15=DE  T19=背光
#==========================================================================

# ---- 红色（RGB565 高位）----
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 } [get_ports {lcd_r[4]}]
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS33 } [get_ports {lcd_r[3]}]
set_property -dict { PACKAGE_PIN V13 IOSTANDARD LVCMOS33 } [get_ports {lcd_r[2]}]
set_property -dict { PACKAGE_PIN U13 IOSTANDARD LVCMOS33 } [get_ports {lcd_r[1]}]
# U12 已让给 I2C 的 SCL；LCD_R3 的信号改到空闲脚 U15（屏上那根线现在由 I2C 驱动）
set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports {lcd_r[0]}]
# T11 已让给 I2C 的 SDA，LCD_R0 不再驱动（悬空不影响显示）

# ---- 绿色（RGB565 高位）----
set_property -dict { PACKAGE_PIN R14 IOSTANDARD LVCMOS33 } [get_ports {lcd_g[5]}]
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports {lcd_g[4]}]
set_property -dict { PACKAGE_PIN P16 IOSTANDARD LVCMOS33 } [get_ports {lcd_g[3]}]
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports {lcd_g[2]}]
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports {lcd_g[1]}]
set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS33 } [get_ports {lcd_g[0]}]
set_property -dict { PACKAGE_PIN V18 IOSTANDARD LVCMOS33 } [get_ports {lcd_g01[1]}]
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports {lcd_g01[0]}]

# ---- 蓝色（RGB565 高位）----
set_property -dict { PACKAGE_PIN R17 IOSTANDARD LVCMOS33 } [get_ports {lcd_b[4]}]
set_property -dict { PACKAGE_PIN R16 IOSTANDARD LVCMOS33 } [get_ports {lcd_b[3]}]
set_property -dict { PACKAGE_PIN U19 IOSTANDARD LVCMOS33 } [get_ports {lcd_b[2]}]
set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports {lcd_b[1]}]
set_property -dict { PACKAGE_PIN W19 IOSTANDARD LVCMOS33 } [get_ports {lcd_b[0]}]
set_property -dict { PACKAGE_PIN W18 IOSTANDARD LVCMOS33 } [get_ports {lcd_b02[2]}]
set_property -dict { PACKAGE_PIN W16 IOSTANDARD LVCMOS33 } [get_ports {lcd_b02[1]}]
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports {lcd_b02[0]}]

# ---- 同步与背光 ----
set_property -dict { PACKAGE_PIN W15 IOSTANDARD LVCMOS33 } [get_ports {lcd_pclk[0]}]
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports {lcd_hsync[0]}]
set_property -dict { PACKAGE_PIN W13 IOSTANDARD LVCMOS33 } [get_ports {lcd_vsync[0]}]
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33 } [get_ports {lcd_de[0]}]
set_property -dict { PACKAGE_PIN T19 IOSTANDARD LVCMOS33 } [get_ports {lcd_bl[0]}]
