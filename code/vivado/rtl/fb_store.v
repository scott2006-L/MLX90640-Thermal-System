//========================================================================
// fb_store.v —— 用 GPIO 并行口接收像素的显存（避开 AXI 的坑）
//
// 为什么不用 AXI：
//   本工程里 axi_bram_ctrl 与 blk_mem_gen 的 BRAM_PORTA 接口虽然"连接成功"，
//   但实测 addra 没有真正连上（所有读写都落在同一地址）。
//   改用 PS 的 EMIO GPIO 直接并行送数据，通路简单、完全可控。
//
// 协议（PS 每次往 GPIO 数据寄存器写一个 32 位数）：
//   bit[15:0]  像素数据 RGB565
//   bit[29:16] 显存地址（0~16383）
//   bit[30]    写入脉冲：PS 先写 0 再写 1，PL 在上升沿锁存
//   bit[31]    蜂鸣器使能（原来在 bit0，现已挪到 bit31）
//
// 时序：PS 写两次（数据+地址为一次，脉冲为一次），每次约 3 个总线周期。
//       768 个像素约需 2300 次写，耗时约 10µs，远小于一帧的时间。
//========================================================================
`timescale 1ns / 1ps

module fb_store (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] gpio_i,       // 来自 PS 的 GPIO_O[31:0]

    // 视频读口（一像素一字，低 16 位有效）
    input  wire [13:0] v_addr,
    output reg  [15:0] v_dout,

    output wire        beep_en       // 送给蜂鸣器方波发生器
);

    // ---------- 从 GPIO 里解出字段 ----------
    wire [15:0] w_data  = gpio_i[15:0];
    wire [13:0] w_addr  = gpio_i[29:16];
    wire        w_pulse = gpio_i[30];

    assign beep_en = gpio_i[31];

    // ---------- 写入使能 ----------
    //
    // 【注意】这里必须是电平触发，不能做上升沿检测！
    //   PS 在 666MHz 下把脉冲拉高又拉低只相隔几纳秒，
    //   而 PL 用 50MHz 采样（周期 20ns），边沿检测会完全采不到这个脉冲，
    //   结果是显存永远写不进去（实测：屏幕全黑）。
    //   改成电平触发后，只要脉冲为高就持续写同一地址（幂等，无副作用），
    //   对 PS 的时序完全没有要求。
    wire wr_en = w_pulse;

    // ---------- 显存：简单双端口 RAM（PS 写 / 视频读，同源时钟）----------
    // 16K × 16bit = 32KB，够放 768 个热力块 + 480×24 的信息区
    reg [15:0] mem [0:16383];

    always @(posedge clk) begin
        if (wr_en) begin
            mem[w_addr] <= w_data;
        end
        v_dout <= mem[v_addr];
    end

endmodule
