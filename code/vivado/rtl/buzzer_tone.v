//========================================================================
// buzzer_tone.v —— 无源压电蜂鸣器方波驱动
//
// 底板蜂鸣器电路（野火 ZYNQ7010 底板原理图 page9）：
//   BEEP(FPGA U14) -- R100 1K --> Q2(SS8050 NPN) --> PZ1 无源压电蜂鸣器 --> 3V3
//   PZ1 两端并 D4(1N4148W) 续流二极管
//
// 要点：
//   1) 引脚是 U14（IO_L11P_T1_SRCC_34），不是相邻的 U15（IO_L11N）
//   2) BEEP 高电平 → Q2 导通 → 蜂鸣器得电
//   3) PZ1 是【无源】压电片，必须 2~4kHz 方波，直流不响
//
// 本模块用 FCLK_CLK0(50MHz) 的 cnt[13] 分频：
//   50MHz / 2^14 = 3051Hz  ← 压电片谐振点附近，声音最响最清脆
//
// en 来自 PS GPIO EMIO bit0（软件只需写一个寄存器位，方波由硬件持续输出，
// 主循环读帧阻塞 750ms 期间蜂鸣器也照常响）
//========================================================================
`timescale 1ns / 1ps

module buzzer_tone #(
    parameter integer DIV_BIT = 13
) (
    input  wire clk,     // FCLK_CLK0 = 50MHz
    input  wire en,      // 1 = 响，0 = 静音
    output wire beep     // 接 BEEP 引脚（U14）
);

    reg [31:0] cnt = 32'd0;

    always @(posedge clk) begin
        cnt <= cnt + 32'd1;
    end

    assign beep = en & cnt[DIV_BIT];

endmodule
