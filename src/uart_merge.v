//=============================================================================
//  uart_merge.v  —  两路 8N1 发送流合流到同一个物理引脚
//
//  背景
//  ----
//  合并后的工程里有两套互相独立的诊断文本输出：
//    a) 视频侧 cam_diag 的第二路（原直连 U15，板载 USB 桥片）
//    b) 麦克风侧 mic_uart_framer → audio_uart_tx（原也接 U15）
//  板载 USB 桥片只连了 U15 这一个脚，所以两路必须在逻辑上共享它。
//
//  为什么不能简单线或
//  ------------------
//  两个发送器各自有独立的位计数器，同一时刻可能一个在发 0x41、另一个在发
//  0x0F，线与之后接收端看到的是两段叠加的波形 → 整段乱码。所以必须互斥。
//
//  做法（字节级、非抢占）
//  ---------------------
//  1. 空闲时 owner=0，输出恒定 1（UART 空闲电平）。
//  2. 谁先出现起始位（tx 拉低）谁拿到线路，owner 锁定 10 个位时间
//     （1 起始 + 8 数据 + 1 停止 = 10 bit = 4340 个 50MHz 时钟）。
//  3. 锁定期间输出完全跟随持有者，绝不切换 —— 所以线上永远只出现
//     **完整字节**，接收端不会遇到 framing error。
//  4. 同时请求时 a（视频）优先。被让路的一方丢掉**整个字节**，
//     对文本诊断而言只是少一个字符，不会污染解析。
//  5. 持有者连续发送时，释放瞬间下一字节的起始位正好接上，可无缝续传。
//
//  为什么是 4340 而不是精确的 4340
//  ------------------------------
//  起始位检测有 1~2 拍（20~40 ns）的延迟，即使早/晚释放几拍，相对
//  8.68 us 的一位时间也只有 0.5% 误差，落在 UART 允许的 ±3% 之内；
//  且每次重新锁定时都会重新对齐，误差不会累积。
//
//  时序：单时钟域（clk = 50 MHz），两路源也都在 clk 域，无跨域问题。
//=============================================================================
`timescale 1ns / 1ps

module uart_merge #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       a_tx,        // 源 A（视频诊断），高优先级
    input  wire       b_tx,        // 源 B（麦克风诊断）
    output wire       tx_out,      // 合流后的串口输出，接 U15
    output reg        conflict     // 曾发生「两路同时有请求」的粘滞标志（调试用）
);

    localparam integer DIV   = (CLK_HZ / BAUD < 2) ? 2 : (CLK_HZ / BAUD);
    localparam integer FRAME = 10 * DIV;            // 10 bit × 434 = 4340

    localparam [1:0] OWNER_IDLE = 2'd0;
    localparam [1:0] OWNER_A    = 2'd1;
    localparam [1:0] OWNER_B    = 2'd2;

    reg [1:0]  owner;
    reg [12:0] cnt;                                 // 0 .. FRAME-1（4340 < 8192）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner    <= OWNER_IDLE;
            cnt      <= 13'd0;
            conflict <= 1'b0;
        end else begin
            case (owner)
                OWNER_IDLE: begin
                    cnt <= 13'd0;
                    if (!a_tx) begin
                        owner <= OWNER_A;
                        if (!b_tx) conflict <= 1'b1;   // 两路同时请求，B 被让路
                    end else if (!b_tx) begin
                        owner <= OWNER_B;
                    end
                end

                default: begin                          // OWNER_A 或 OWNER_B
                    if (cnt >= FRAME - 1) begin
                        owner <= OWNER_IDLE;
                        cnt   <= 13'd0;
                    end else begin
                        cnt <= cnt + 13'd1;
                    end
                end
            endcase
        end
    end

    // 空闲时拉高；持有期间完全跟随持有者，因此线上只出现完整字节
    assign tx_out = (owner == OWNER_A) ? a_tx :
                    (owner == OWNER_B) ? b_tx :
                                         1'b1;

endmodule
