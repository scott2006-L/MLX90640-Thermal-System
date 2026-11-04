//========================================================================
// lcd_rgb.v —— 4.3 寸 480×272 RGB 屏驱动 + 热力图显示
//
// 时序：HSW=41 HBP=8 HACT=480 HFP=4 / VSW=10 VBP=2 VACT=272 VFP=4
//       PCLK = 50MHz/6 = 8.33MHz → 帧率 ≈ 54Hz，HSYNC/VSYNC 低有效
//
// 显存（BRAM 32 位 × 16384 = 64KB，PS 通过 AXI 写入，基址 0x40000000）
//   为保证地址换算绝对可靠：**一个 32 位字只放 1 个像素**，只用低 16 位
//     字 0x0000 ~ 0x02FF ：768 个热力块颜色，下标 = 行*32 + 列
//     字 0x0400 ~ 0x34FF ：信息区（480×24），留给后面的文字显示
//   软件写：*(uint32_t*)(0x40000000 + i*4) = RGB565;
//
// 画面：y =   0 ~ 215  热力图（32×24 色块，每块 15×9）
//       y = 216 ~ 271  信息区（暂为黑色）
//
// 颜色只驱动 RGB565 高位：R7~R3、G7~G2、B7~B3；低位驱 0
//   注意 T10/T12（R1/R2）被 I2C 占用，本模块不驱动
//========================================================================
`timescale 1ns / 1ps

module lcd_rgb (
    input  wire        clk,          // FCLK_CLK0 = 50MHz
    input  wire        rst_n,

    // 显存读口（接 fb_store 的 v_dout，一像素一字，低 16 位有效）
    output wire [13:0] bram_addr,
    input  wire [15:0] bram_dout,

    output wire [4:0]  o_r,
    output wire        o_r0,
    output wire [5:0]  o_g,
    output wire [1:0]  o_g01,
    output wire [4:0]  o_b,
    output wire [2:0]  o_b02,

    output wire        o_pclk,
    output wire        o_hsync,
    output wire        o_vsync,
    output wire        o_de,
    output wire        o_bl
);

    // ---------------- 时序 ----------------
    localparam HSW  = 41;
    localparam HBP  = 8;
    localparam HACT = 480;
    localparam HFP  = 4;
    localparam HTOT = HSW + HBP + HACT + HFP;    // 533

    localparam VSW  = 10;
    localparam VBP  = 2;
    localparam VACT = 272;
    localparam VFP  = 4;
    localparam VTOT = VSW + VBP + VACT + VFP;    // 288

    localparam DIV  = 6;

    localparam HEAT_H     = 216;
    localparam HEAT_H10   = 10'd216;     // 参数不能直接做位选择，这里另定义一个 10 位常量
    localparam CELL_W     = 15;
    localparam CELL_H     = 9;
    localparam TEXT_BASE  = 14'd1024;            // 信息区起始字

    // ---------------- 像素时钟 ----------------
    reg [2:0] div_cnt = 3'd0;
    reg       pclk_r  = 1'b0;

    always @(posedge clk) begin
        if (div_cnt == (DIV - 1)) begin
            div_cnt <= 3'd0;
            pclk_r  <= ~pclk_r;
        end else begin
            div_cnt <= div_cnt + 3'd1;
        end
    end

    // ---------------- 行列计数 ----------------
    reg [9:0] hcnt = 10'd0;
    reg [9:0] vcnt = 10'd0;

    always @(posedge pclk_r or negedge rst_n) begin
        if (!rst_n) begin
            hcnt <= 10'd0;
            vcnt <= 10'd0;
        end else begin
            if (hcnt == (HTOT - 1)) begin
                hcnt <= 10'd0;
                if (vcnt == (VTOT - 1)) vcnt <= 10'd0;
                else                    vcnt <= vcnt + 10'd1;
            end else begin
                hcnt <= hcnt + 10'd1;
            end
        end
    end

    wire h_active = (hcnt >= HSW + HBP) && (hcnt < HSW + HBP + HACT);
    wire v_active = (vcnt >= VSW + VBP) && (vcnt < VSW + VBP + VACT);
    wire active   = h_active && v_active;

    wire [9:0] x = hcnt - (HSW + HBP);      // 0~479
    wire [9:0] y = vcnt - (VSW + VBP);      // 0~271

    // ---------------- 显存地址（一像素一字） ----------------
    // 地址用「下一拍」的坐标，配合 BRAM 一拍读延迟
    reg [9:0] x_d1   = 10'd0;
    reg [9:0] y_d1   = 10'd0;
    reg       act_d1 = 1'b0;

    always @(posedge pclk_r or negedge rst_n) begin
        if (!rst_n) begin
            x_d1   <= 10'd0;
            y_d1   <= 10'd0;
            act_d1 <= 1'b0;
        end else begin
            x_d1   <= x;
            y_d1   <= y;
            act_d1 <= active;
        end
    end

    wire        is_heat = (y_d1 < HEAT_H);
    wire [4:0]  cell_c  = x_d1 / CELL_W;                // 0~31
    wire [4:0]  cell_r  = y_d1 / CELL_H;                // 0~23
    wire [9:0]  heat_idx = {cell_r, cell_c};            // 行*32+列 0~767

    // 信息区：从信息带里取中间 32 行（480×32 = 15360 个字），
    // 加上 1024 的偏移正好落在 16384 个字以内，不浪费也绝不越界。
    wire [9:0]  ty      = y_d1 - HEAT_H10;                        // 0~55
    wire        in_text = (ty >= 10'd12) && (ty < 10'd44);
    wire [14:0] text_idx = (ty - 10'd12) * 10'd480 + x_d1;        // 0~15359

    wire [13:0] addr_heat = {4'd0, heat_idx};                      // 0~767
    wire [13:0] addr_text = TEXT_BASE + text_idx[13:0];            // 1024~16383

    assign bram_addr = is_heat ? addr_heat : (in_text ? addr_text : 14'd16383);

    // 取色：只取低 16 位（一像素一字）
    reg [15:0] color_pix = 16'h0000;

    always @(posedge pclk_r or negedge rst_n) begin
        if (!rst_n) color_pix <= 16'h0000;
        else        color_pix <= bram_dout;
    end

    // ---------------- 边框与移动标记 ----------------
    reg [8:0] frame_cnt = 9'd0;

    always @(posedge pclk_r or negedge rst_n) begin
        if (!rst_n) frame_cnt <= 9'd0;
        else if (hcnt == 10'd0 && vcnt == 10'd0) frame_cnt <= frame_cnt + 9'd1;
    end

    wire [8:0] mark_x = (frame_cnt * 9'd9) % 9'd480;
    wire       marker = (x == mark_x);
    wire       border = (x < 10'd2 || x >= 10'd478 || y < 10'd2 || y >= 10'd270);

    // ---------------- 输出 ----------------
    reg [15:0] rgb_r = 16'h0000;
    reg        hs_r, vs_r, de_r;

    always @(posedge pclk_r or negedge rst_n) begin
        if (!rst_n) begin
            rgb_r <= 16'h0000;
            hs_r  <= 1'b1;
            vs_r  <= 1'b1;
            de_r  <= 1'b0;
        end else begin
            if (border)      rgb_r <= 16'hFFFF;
            else if (marker) rgb_r <= 16'hF800;
            else             rgb_r <= color_pix;
            hs_r <= ~(hcnt < HSW);
            vs_r <= ~(vcnt < VSW);
            de_r <= active;
        end
    end

    assign o_r     = rgb_r[15:11];
    assign o_g     = rgb_r[10:5];
    assign o_b     = rgb_r[4:0];
    assign o_r0    = 1'b0;
    assign o_g01   = 2'b00;
    assign o_b02   = 3'b000;

    assign o_pclk  = ~pclk_r;
    assign o_hsync = hs_r;
    assign o_vsync = vs_r;
    assign o_de    = de_r;
    assign o_bl    = 1'b1;

endmodule
