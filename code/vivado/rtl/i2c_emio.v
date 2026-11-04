//==========================================================================
// i2c_emio.v  ——  PS I2C0 EMIO 的三态缓冲（IOBUF）封装
//
// Zynq PS7 的 I2C0 通过 EMIO 引出来时是 6 个信号（sda/scl 各自的 i/o/t），
// 而板上的物理引脚是 1 根双向线，所以必须用 IOBUF 把 i/o/t 合成一个 inout。
//
// 本版新增两个输出：
//   iic_scl_level / iic_sda_level  —— 引脚的实际电平（IOBUF 的 O 端），
//   引到 PS 的 GPIO 输入，软件就能直接读到两根线当前是高还是低，
//   用来诊断"总线被谁拉低 / 是否推挽冲突"。
//==========================================================================
module i2c_emio (
    // ---- 连到 PS7 的 I2C0 EMIO（注意方向：I 是"给 PS 读"，O/T 是"PS 输出"）----
    output wire iic_0_scl_i,
    input  wire iic_0_scl_o,
    input  wire iic_0_scl_t,
    output wire iic_0_sda_i,
    input  wire iic_0_sda_o,
    input  wire iic_0_sda_t,

    // ---- 物理引脚（连到顶层端口，XDC 约束）----
    inout  wire iic_scl,
    inout  wire iic_sda,

    // ---- 引脚实际电平（给 PS 的 GPIO 输入做诊断）----
    output wire iic_scl_level,
    output wire iic_sda_level
);

    IOBUF scl_buf (
        .I  (iic_0_scl_o),
        .O  (iic_0_scl_i),
        .T  (iic_0_scl_t),
        .IO (iic_scl)
    );

    IOBUF sda_buf (
        .I  (iic_0_sda_o),
        .O  (iic_0_sda_i),
        .T  (iic_0_sda_t),
        .IO (iic_sda)
    );

    assign iic_scl_level = iic_0_scl_i;
    assign iic_sda_level = iic_0_sda_i;

endmodule
