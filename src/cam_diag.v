//=============================================================================
//  cam_diag —— 三目显示诊断观测器（P0 第 0 步：先插观测点，测基线）
//
//  目的：把散在各时钟域里的健康状态聚合成周期性 HSPI 包，经 CH569 → USB-CDC
//        送到 PC（COM5），为三目显示优化提供可测数据（docs/TRIPLE_CAM_PERF_PLAN.md
//        中的 K8~K12 全部依赖本模块）。
//
//  时钟域：
//    clk           50 MHz   —— 包发送、EXM_HSPI、clk 域直读信号
//    cam1_clk      cam1 的 pclk÷2 —— 全部跨域计数器（帧号/错位/FIFO 溢出）
//
//  观测量：
//    fc1/fc2/fc3   三路帧计数（cam1: vsync 上升沿；cam2/3: 已同步的单拍脉冲）
//    ff2/ff3       cam2/3 FIFO 满脉冲计数（溢出/饥饿迹象）
//    ma2/ma3       行错位计数：显示侧行首(pixel_counter==0 且有效)时
//                  行缓冲写指针应为 0，否则计一次
//    lut_index_*   I2C 配置查找表索引（哨兵失效时会看到 0..252 循环）
//    i2c_err_*     三路 I2C 配置错误锁存
//    offset        平移取景偏移（12 bit）
//    snap_valid    cam1 域是否给出过快照（=0 说明 cmos_16bit_clk 没在跑 ⇒ cam1 未出图）
//
//  CDC：clk 域翻转 req → cam1 域 2FF 同步并锁存快照 → 翻转 ack → clk 域读走。
//       快照间隔(≈0.34 s) ≫ 同步延迟，快照寄存器在读走之前保持不变，读取安全。
//
//  包格式（512 字节，256×16bit；高字节 = 标记 5A/A5 交替，与 P0/P1 一致，
//          HSPI 线上字节序低字节在前）：
//    idx  内容
//    0    0xA5                      同步
//    1..2 pkt_cnt                   包计数
//    3    {err3,err2,err1,5'b0}     I2C 错误
//    4..6 lut_index cam1/2/3 低 8 位
//    7..9   fc1 (24bit, LSB 先)
//    10..11 fc2 (16bit)
//    12..13 fc3 (16bit)
//    14..17 ff2,ff3,ma2,ma3 (各 8bit)
//    18..19 offset[7:0], {3'b0,offset[12:8]}
//    20   {7'b0, snap_valid}
//    21   0x5A                      保留标记
//    其余 0x00
//=============================================================================
module cam_diag (
    // ---- 50 MHz 主域 ----
    input  wire        clk,
    input  wire        rst_n,
    // ---- cam1 域（cmos_16bit_clk）----
    input  wire        cam1_clk,
    input  wire        cam1_vs,           // cam1 vsync（电平）
    input  wire        cam2_vs_pulse,     // cam2 vsync 已同步单拍脉冲
    input  wire        cam3_vs_pulse,
    input  wire        cam2_fifo_full,
    input  wire        cam3_fifo_full,
    input  wire [10:0] cam2_wr_ptr,
    input  wire [10:0] cam3_wr_ptr,
    input  wire [10:0] pixel_counter,
    input  wire        pixel_en,          // = cmos_16bit_wr
    // ---- clk 域直读 ----
    input  wire [9:0]  lut_index_cam1,
    input  wire [9:0]  lut_index_cam2,
    input  wire [9:0]  lut_index_cam3,
    input  wire        i2c_err_cam1,
    input  wire        i2c_err_cam2,
    input  wire        i2c_err_cam3,
    input  wire [12:0] horizontal_offset,
    // ---- HSPI（经 CH569 → USB-CDC 到 PC）----
    output wire        hspi_htclk,
    output wire        hspi_htreq,
    input  wire        hspi_htrdy,
    output wire        hspi_htvld,
    inout  wire [15:0] hspi_hd,
    // ---- UART 文本行（115200 8N1）：U15 走板载桥片，G11 备用 CH340 ----
    output wire        diag_uart_tx,
    output wire        diag_uart_tx2
);

    //=========================================================================
    //  cam1 域：计数器
    //=========================================================================
    reg [23:0] fc1;
    reg [15:0] fc2, fc3;
    reg [7:0]  ff2, ff3, ma2, ma3;
    reg        cam1_vs_d, cam2_ff_d, cam3_ff_d;

    always @(posedge cam1_clk or negedge rst_n) begin
        if (!rst_n) begin
            fc1 <= 24'd0;  fc2 <= 16'd0;  fc3 <= 16'd0;
            ff2 <= 8'd0;   ff3 <= 8'd0;   ma2 <= 8'd0;  ma3 <= 8'd0;
            cam1_vs_d <= 1'b0;  cam2_ff_d <= 1'b0;  cam3_ff_d <= 1'b0;
        end else begin
            cam1_vs_d <= cam1_vs;
            if (cam1_vs & ~cam1_vs_d)                 fc1 <= fc1 + 24'd1;
            if (cam2_vs_pulse)                        fc2 <= fc2 + 16'd1;
            if (cam3_vs_pulse)                        fc3 <= fc3 + 16'd1;

            cam2_ff_d <= cam2_fifo_full;
            if (cam2_fifo_full & ~cam2_ff_d)          ff2 <= ff2 + 8'd1;
            cam3_ff_d <= cam3_fifo_full;
            if (cam3_fifo_full & ~cam3_ff_d)          ff3 <= ff3 + 8'd1;

            // 显示侧行首：首像素时行缓冲写指针必须为 0
            if (pixel_en && (pixel_counter == 11'd0)) begin
                if (cam2_wr_ptr != 11'd0)             ma2 <= ma2 + 8'd1;
                if (cam3_wr_ptr != 11'd0)             ma3 <= ma3 + 8'd1;
            end
        end
    end

    //=========================================================================
    //  快照握手（clk → cam1）
    //=========================================================================
    wire diag_req;                          // clk 域翻转的请求电平
    wire diag_ack;                          // cam1 域翻转的应答电平

    reg  snap_req_s0, snap_req_s1, snap_req_s2;
    reg  ack_toggle;
    reg  [23:0] s_fc1;
    reg  [15:0] s_fc2, s_fc3;
    reg  [7:0]  s_ff2, s_ff3, s_ma2, s_ma3;
    reg  snap_valid;

    always @(posedge cam1_clk or negedge rst_n) begin
        if (!rst_n) begin
            snap_req_s0 <= 1'b0;  snap_req_s1 <= 1'b0;  snap_req_s2 <= 1'b0;
            ack_toggle  <= 1'b0;  snap_valid   <= 1'b0;
        end else begin
            snap_req_s0 <= diag_req;
            snap_req_s1 <= snap_req_s0;
            snap_req_s2 <= snap_req_s1;
            if (snap_req_s1 != snap_req_s2) begin
                s_fc1 <= fc1;   s_fc2 <= fc2;   s_fc3 <= fc3;
                s_ff2 <= ff2;   s_ff3 <= ff3;   s_ma2 <= ma2;   s_ma3 <= ma3;
                ack_toggle  <= ~ack_toggle;
                snap_valid  <= 1'b1;
            end
        end
    end
    assign diag_ack = ack_toggle;

    //=========================================================================
    //  clk 域：快照请求节拍 + 应答同步 + 锁存
    //=========================================================================
    reg [23:0] snap_timer;
    reg        req_toggle_r;
    assign diag_req = req_toggle_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snap_timer   <= 24'd0;
            req_toggle_r <= 1'b0;
        end else begin
            snap_timer <= snap_timer + 24'd1;
            if (&snap_timer) req_toggle_r <= ~req_toggle_r;   // ≈0.34 s 一次
        end
    end

    reg ack_s0, ack_s1;
    reg [23:0] r_fc1;
    reg [15:0] r_fc2, r_fc3;
    reg [7:0]  r_ff2, r_ff3, r_ma2, r_ma3, r_lut1, r_lut2, r_lut3, r_err, r_sv;
    reg [12:0] r_off;
    reg [31:0] pkt_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_s0 <= 1'b0;  ack_s1 <= 1'b0;  pkt_cnt <= 32'd0;
            r_fc1 <= 24'd0;  r_fc2 <= 16'd0;  r_fc3 <= 16'd0;
            r_ff2 <= 8'd0;   r_ff3 <= 8'd0;
            r_ma2 <= 8'd0;   r_ma3 <= 8'd0;
            r_lut1 <= 8'd0;  r_lut2 <= 8'd0;  r_lut3 <= 8'd0;
            r_err <= 8'd0;   r_sv  <= 8'd0;   r_off <= 13'd0;
        end else begin
            ack_s0 <= diag_ack;
            ack_s1 <= ack_s0;
            if (ack_s0 != ack_s1) begin            // 新快照到达
                r_fc1 <= s_fc1;  r_fc2 <= s_fc2;  r_fc3 <= s_fc3;
                r_ff2 <= s_ff2;  r_ff3 <= s_ff3;
                r_ma2 <= s_ma2;  r_ma3 <= s_ma3;
                r_lut1 <= lut_index_cam1[7:0];
                r_lut2 <= lut_index_cam2[7:0];
                r_lut3 <= lut_index_cam3[7:0];
                r_err  <= {5'b0, i2c_err_cam3, i2c_err_cam2, i2c_err_cam1};
                r_off  <= horizontal_offset;
                r_sv   <= {7'b0, snap_valid};
                pkt_cnt <= pkt_cnt + 32'd1;
            end
        end
    end

    //=========================================================================
    //  HSPI：clk/4 = 12.5 MHz，每 2^23 hspi_clk ≈ 0.67 s 发一包
    //=========================================================================
    reg [1:0]  div4;
    reg [22:0] tick;
    reg        tick_d;
    reg        rst_n_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div4 <= 2'd0;  tick <= 23'd0;  tick_d <= 1'b0;  rst_n_d <= 1'b0;
        end else begin
            div4   <= div4 + 2'd1;
            tick   <= tick + 23'd1;
            tick_d <= tick[22];
            rst_n_d <= 1'b1;
        end
    end

    wire hspi_clk = div4[1];
    wire tx_trig  = tick[22] & ~tick_d;

    // hspi_clk 域的包计数与触发同步
    reg [15:0] pkt_cnt_h;
    reg        trig_d;
    always @(posedge hspi_clk or negedge rst_n_d) begin
        if (!rst_n_d) begin
            pkt_cnt_h <= 16'd0;  trig_d <= 1'b0;
        end else begin
            trig_d <= tx_trig;
            if (tx_trig & ~trig_d) pkt_cnt_h <= pkt_cnt_h + 16'd1;
        end
    end
    wire tx_act = tx_trig & ~trig_d;

    //=========================================================================
    //  净荷
    //=========================================================================
    wire [8:0] ram_addr;
    wire [8:0] idx  = ram_addr - 9'd1;
    wire       odd  = idx[0];
    wire [7:0] mark = odd ? 8'hA5 : 8'h5A;

    reg [7:0] dbyte;
    always @(*) begin
        case (idx)
            9'd0:    dbyte = 8'hA5;                                   // 同步
            9'd1:    dbyte = pkt_cnt_h[7:0];
            9'd2:    dbyte = pkt_cnt_h[15:8];
            9'd3:    dbyte = r_err;                                   // I2C 错误
            9'd4:    dbyte = r_lut1;
            9'd5:    dbyte = r_lut2;
            9'd6:    dbyte = r_lut3;
            9'd7:    dbyte = r_fc1[7:0];
            9'd8:    dbyte = r_fc1[15:8];
            9'd9:    dbyte = r_fc1[23:16];
            9'd10:   dbyte = r_fc2[7:0];
            9'd11:   dbyte = r_fc2[15:8];
            9'd12:   dbyte = r_fc3[7:0];
            9'd13:   dbyte = r_fc3[15:8];
            9'd14:   dbyte = r_ff2;
            9'd15:   dbyte = r_ff3;
            9'd16:   dbyte = r_ma2;
            9'd17:   dbyte = r_ma3;
            9'd18:   dbyte = r_off[7:0];
            9'd19:   dbyte = {3'b000, r_off[12:8]};
            9'd20:   dbyte = r_sv;                                    // snap_valid
            9'd21:   dbyte = 8'h5A;                                   // 保留标记
            default: dbyte = 8'h00;
        endcase
    end

    wire [15:0] payload_word = {mark, dbyte};
    wire [15:0] resv;

    //=========================================================================
    //  UART 备用通路：每次快照发一行十六进制文本（115200 8N1，约 31 字节）
    //  行格式：D <lut1:2> <fc1:6> <ff2:2> <ff3:2> <ma2:2> <ma3:2> <off:3> <err:1>\r\n
    //=========================================================================
    function [7:0] h2a;                     // 4bit -> ASCII hex
        input [3:0] n;
        begin
            h2a = (n < 4'd10) ? (8'h30 + {4'b0, n}) : (8'h37 + {4'b0, n});
        end
    endfunction

    reg [5:0]  bi;
    reg        sending;
    reg        tx_start_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bi <= 6'd0;  sending <= 1'b0;  tx_start_r <= 1'b0;
        end else begin
            tx_start_r <= 1'b0;
            if (ack_s0 != ack_s1 && !sending) begin
                bi      <= 6'd0;
                sending <= 1'b1;
            end else if (sending) begin
                if (!uart_busy) begin
                    tx_start_r <= 1'b1;
                    if (bi == 6'd30) sending <= 1'b0;
                    bi <= bi + 6'd1;
                end
            end
        end
    end

    reg [7:0] line_byte;
    always @(*) begin
        case (bi)
            6'd0:    line_byte = 8'h44;                              // 'D'
            6'd1:    line_byte = 8'h20;
            6'd2:    line_byte = h2a(r_lut1[7:4]);
            6'd3:    line_byte = h2a(r_lut1[3:0]);
            6'd4:    line_byte = 8'h20;
            6'd5:    line_byte = h2a(r_fc1[23:20]);
            6'd6:    line_byte = h2a(r_fc1[19:16]);
            6'd7:    line_byte = h2a(r_fc1[15:12]);
            6'd8:    line_byte = h2a(r_fc1[11:8]);
            6'd9:    line_byte = h2a(r_fc1[7:4]);
            6'd10:   line_byte = h2a(r_fc1[3:0]);
            6'd11:   line_byte = 8'h20;
            6'd12:   line_byte = h2a(r_ff2[7:4]);
            6'd13:   line_byte = h2a(r_ff2[3:0]);
            6'd14:   line_byte = 8'h20;
            6'd15:   line_byte = h2a(r_ff3[7:4]);
            6'd16:   line_byte = h2a(r_ff3[3:0]);
            6'd17:   line_byte = 8'h20;
            6'd18:   line_byte = h2a(r_ma2[7:4]);
            6'd19:   line_byte = h2a(r_ma2[3:0]);
            6'd20:   line_byte = 8'h20;
            6'd21:   line_byte = h2a(r_ma3[7:4]);
            6'd22:   line_byte = h2a(r_ma3[3:0]);
            6'd23:   line_byte = 8'h20;
            6'd24:   line_byte = h2a(r_off[11:8]);
            6'd25:   line_byte = h2a(r_off[7:4]);
            6'd26:   line_byte = h2a(r_off[3:0]);
            6'd27:   line_byte = 8'h20;
            6'd28:   line_byte = h2a(r_err[3:0]);
            6'd29:   line_byte = 8'h0D;
            6'd30:   line_byte = 8'h0A;
            default: line_byte = 8'h20;
        endcase
    end

    wire uart_busy;
    wire [7:0] uart_d = line_byte;

    // 两路输出同一内容：U15（板载桥片，同事已验证）+ G11（DONE 脚释放，CH340 备用）
    // 两实例同 clk 同数据同启动，busy 时序完全一致，用第一路的 busy 做握手即可
    uart_tx #(
        .CLK_HZ (50_000_000),
        .BAUD   (115200)
    ) u_diag_uart (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (uart_d),
        .tx_start (tx_start_r),
        .tx       (diag_uart_tx),
        .busy     (uart_busy)
    );

    uart_tx #(
        .CLK_HZ (50_000_000),
        .BAUD   (115200)
    ) u_diag_uart2 (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (uart_d),
        .tx_start (tx_start_r),
        .tx       (diag_uart_tx2),
        .busy     ()
    );

    EXM_HSPI #(
        .TX_LEN  (12'd512 - 1),
        .HSPI_UDF(26'h000ffff)
    ) M_HSPI (
        .HTCLK      (hspi_htclk),
        .HTREQ      (hspi_htreq),
        .HTRDY      (hspi_htrdy),
        .HTVLD      (hspi_htvld),
        .HTD        ({resv, hspi_hd}),
        .HTOE       (),
        .tx_act     (tx_act),
        .HRCLK      (1'b0),
        .HRACT      (1'b0),
        .HTACK      (),
        .HRVLD      (1'b0),
        .HRD        (32'd0),
        .rstn       (rst_n_d),
        .clk        (hspi_clk),
        .dat_mod    (2'b01),
        .ram_clk    (),
        .ram_csn    (),
        .ram_wen    (),
        .ram_addr   (ram_addr),
        .ram_rdata  ({16'h0000, payload_word}),
        .ram_wdata  ()
    );

endmodule
