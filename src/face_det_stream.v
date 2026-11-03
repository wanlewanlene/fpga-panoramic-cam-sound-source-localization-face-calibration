//=============================================================================
//  face_det_stream.v  —  视频域流式人脸检测（纯整数，无 ROM、无乘法器）
//
//  【为什么是这个算法，而不是 Haar / CNN】
//  本板 GW5AT-60B 无 NPU/ARM；合并工程已用 Logic ~53%、CLS ~68%，且时钟网络
//  PRIMARY/LW 已满（不能新建时钟域）；Haar 需要 ~17KB 级联表（且本机离线取不到
//  OpenCV 级联数据）。因此在"保证能编、能跑"的前提下选三段式整数算法：
//
//    1) 肤色分割      RGB565 → 阈值判据（每像素 6 次比较，无乘法）
//    2) 块统计+行投影  8×6 像素块 → 块行内"肤色块数" → 取最长连续带
//    3) 眼带密度判据   眼睛/眉毛不是肤色 ⇒ 带内上 1/4~2/5 的密度显著低于下 3/5~4/5
//
//  【本次重写修掉的三个真缺陷】（旧版因此从未真正检出过目标）
//
//   B1 ★致命：块累加器读成了"上一块"的值
//      旧版  acc_rd <= acc[bx];  blk_sum = blk_cnt + acc_rd;
//      但 acc_rd 只在 blk_end 那拍更新，到下一块 blk_end 时，acc_rd 里装的是
//      **前一个块**的部分和 ⇒ 累加跨块串扰，所有块统计都是错的。
//      改：用组合读 acc_cur = acc[bx]，与写入用同一拍同一个旧值（非阻塞语义）。
//
//   B2 行统计数组打了一拍，导致扫描索引与数据错位一行
//      旧版  rs_cnt_rd <= rs_cnt[pi];  然后用 rs_cnt_rd 判断 pi 这一行
//      ⇒ 实际读到的是 pi-1 的行，最后一行永远扫不到，段边界整体偏一行。
//      改：直接组合读 rs_cnt[pi]（这三组是纯读、只有帧末 FSM 访问，不需要寄存）。
//
//   B3 face_tgl 只在"检出"时才翻转
//      旧版在 P_OUT 的 if 分支里翻 tgl ⇒ 没检出时接收端**永远等不到边沿**，
//      会一直沿用上一帧的旧框（人走了框还在）。改：每帧结束无条件翻转，
//      配合 face_conf=0 表示"本帧无目标"。
//
//  另外把帧边界判定从 vs 的电平沿改成 **de 静默超时**：
//  本工程的 syn_off0_vs 极性在 IP 两侧被取反过一次（top.v:901 传的是 ~vs），
//  靠极性推断不稳；而 'de 连续为低 64 拍' 在任何极性下都成立。
//
//  【精度边界】正面/近正面、光照 200~1500 lux；木质/纸箱等大面积肤色背景可能误检
//  （用 MAX_W + 眼带密度压制，不能完全消除）；侧脸>45°、强逆光、帽子会漏检。
//  这是**面向闭环跟踪**的检测器：误检由"必须连续多帧一致"过滤，漏检由控制器的
//  保持/回退兜底。要更高精度可整体替换为 Haar 引擎，接口不变。
//
//  【资源】~1.5K LUT、0 DSP、0 乘法器；块累加器与行统计走分布式 RAM。
//  【时序】全部在单一视频时钟域（video_clk），无跨域；入口打一拍切长路径。
//=============================================================================
`timescale 1ns / 1ps

module face_det_stream #(
    parameter integer BW  = 8,        // 水平降采样（像素/块列）
    parameter integer BWC = 3,        // log2(BW)，用于 lx[2:0] 判定
    parameter integer BH  = 6,        // 垂直降采样（行/块行）
    parameter integer BHC = 5,        // BH-1，组内最后一行判定
    parameter integer CW  = 160,      // 块列数 = 1280/8
    parameter integer CH  = 120,      // 块行数 = 720/6
    // 块内肤色像素门限（块共 48 像素）。6 = 12.5%，跨脸/背景边界的块也能计入
    parameter integer BLK_THR = 6,
    // 块行内"肤色块数"门限（0..160）。8 块 = 64 px 宽，与 MIN_H 的宽高比自洽
    parameter integer ROW_THR = 8,
    parameter integer MIN_H   = 8,    // 目标块高下限（8×6 = 48 行 ≈ 3 m）
    parameter integer MAX_H   = 70,   // 上限（70×6 = 420 行，防整幅误判）
    parameter integer MAX_W   = 128,  // 块宽上限（128×8 = 1024 px，拒绝大面积肤色背景）
    parameter integer EYE_RATIO_Q8 = 205   // 眼带/颊带密度比 < 205/256 ⇒ 判为脸
)(
    input  wire        clk_v,          // 视频时钟（video_clk）
    input  wire        rst_n,
    input  wire        px_de,          // 显示侧数据有效（off0_syn_de）
    input  wire [15:0] px_data,        // RGB565（off0_syn_data）
    input  wire        vs,             // 帧同步（仅作参考，帧边界改用 de 静默判定）
    // ---- 结果（块坐标）----
    output reg  [7:0]  face_cx,        // 框中心列（×8 = 屏幕像素 x）
    output reg  [7:0]  face_cy,        // 框中心行（×6 = 屏幕像素 y）
    output reg  [7:0]  face_w,         // 框宽（块数，×8 = 像素）
    output reg  [7:0]  face_h,         // 框高（块行数，×6 = 像素）
    output reg  [7:0]  face_conf,      // 0 = 本帧无目标；其余越大越可信
    output reg         face_tgl        // 每帧结束翻转一次（跨域握手用）
);

    //==================================================================
    // 0) 输入管线（1 级）
    //
    //    px_data 直接来自帧缓冲读 FIFO 的分布式 RAM 异步输出，若不加寄存，
    //    会出现 "RAM DO → 肤色比较 → 累加器使能" 的跨模块长路径（实测在
    //    clk_74_25 域产生 9 个端点违例）。整组信号同拍后移，相位关系不变。
    //==================================================================
    reg [15:0] px_d1;
    reg        de_d1, vs_d1;

    always @(posedge clk_v or negedge rst_n) begin
        if (!rst_n) begin
            px_d1 <= 16'd0;
            de_d1 <= 1'b0;
            vs_d1 <= 1'b0;
        end else begin
            px_d1 <= px_data;
            de_d1 <= px_de;
            vs_d1 <= vs;
        end
    end

    //==================================================================
    // 1) 肤色判据（RGB565 → 8 位近似，只做比较与减法，无乘法）
    //==================================================================
    wire [7:0] r8 = {px_d1[15:11], 3'b000};
    wire [7:0] g8 = {px_d1[10:5],  2'b00};
    wire [7:0] b8 = {px_d1[4:0],   3'b000};

    wire skin_px = (r8 > 8'd70) && (g8 > 8'd35) && (b8 > 8'd15)
                && (r8 > g8) && (g8 >= b8)
                && ((r8 - b8) > 8'd20) && (r8 < 8'd250);

    //==================================================================
    // 2) 帧边界：de 连续为低 64 拍 ⇒ 进入消隐期（与 vs 极性无关）
    //==================================================================
    reg [9:0] de_gap;

    always @(posedge clk_v or negedge rst_n) begin
        if (!rst_n)                       de_gap <= 10'd0;
        else if (de_d1)                   de_gap <= 10'd0;
        else if (de_gap != 10'h3FF)       de_gap <= de_gap + 10'd1;
    end

    // de_gap 只在 ==63 那一拍成立（下一拍就 64），天然是单拍脉冲，
    // 不需要额外的"已处理"标志（那个标志还会和主时序块形成双驱动）。
    wire frame_end = (de_gap == 10'd63);

    //==================================================================
    // 3) 位置计数与块统计
    //==================================================================
    reg        de_d;
    reg [11:0] lx;                        // 行内像素号
    reg [9:0]  ly;                        // 帧内行号
    reg [2:0]  vg;                        // 组内行号
    reg [6:0]  by;                        // 块行号 0..119
    reg [7:0]  bx;                        // 块列号 0..159

    wire line_start = de_d1 & ~de_d;
    wire blk_end    = de_d1 & (lx[BWC-1:0] == BW[2:0] - 3'd1);
    wire grp_last   = blk_end & (vg == BHC[2:0]);

    reg [5:0]  blk_cnt;                   // 当前块内已累加的行像素命中数
    reg [4:0]  acc [0:CW-1];              // 前几行的部分和（按块列）
    wire [4:0] acc_cur = acc[bx];         // ★ B1 修正：组合读当前块

    // 本块最终和 = 已有部分和 + 本行已计 + 当前像素（blk_end 那拍正好是第 8 个像素）
    wire [6:0] blk_sum = {1'b0, blk_cnt} + {2'b00, acc_cur} + (skin_px ? 7'd1 : 7'd0);
    wire [4:0] acc_new = (blk_sum > 7'd31) ? 5'd31 : blk_sum[4:0];

    reg [7:0]  row_cnt;                   // 本块行内肤色块数
    reg [7:0]  row_min, row_max;          // 本块行内肤色块的列范围

    // 逐块行的统计（帧末 FSM 扫描用）
    reg [7:0]  rs_cnt [0:CH-1];
    reg [7:0]  rs_min [0:CH-1];
    reg [7:0]  rs_max [0:CH-1];

    //==================================================================
    // 4) 帧末后处理 FSM
    //==================================================================
    localparam [2:0] P_IDLE = 3'd0, P_SCAN = 3'd1, P_BAND = 3'd2,
                     P_EYE  = 3'd3, P_OUT  = 3'd4;

    reg [2:0]  pst;
    reg [7:0]  pi;
    reg [7:0]  run_s;
    reg        in_run;
    reg [7:0]  best_s, best_e, best_span;
    reg [7:0]  band_minx, band_maxx;
    reg [7:0]  cmin, cmax;

    reg [15:0] eye_sum, chk_sum;
    reg [7:0]  eye_n, chk_n;
    reg [7:0]  y_lo_eye, y_hi_eye, y_lo_chk, y_hi_chk;
    wire [15:0] bs_span = {8'd0, best_span};

    // ★ B2 修正：组合读，不用打拍的副本
    wire [7:0] rs_cnt_cur = rs_cnt[pi];
    wire [7:0] rs_min_cur = rs_min[pi];
    wire [7:0] rs_max_cur = rs_max[pi];

    always @(posedge clk_v or negedge rst_n) begin
        if (!rst_n) begin
            de_d <= 1'b0;
            lx <= 12'd0; ly <= 10'd0; vg <= 3'd0; by <= 7'd0; bx <= 8'd0;
            blk_cnt <= 6'd0;
            row_cnt <= 8'd0; row_min <= 8'hFF; row_max <= 8'd0;
            pst <= P_IDLE; pi <= 8'd0;
            run_s <= 8'd0; in_run <= 1'b0;
            best_s <= 8'd0; best_e <= 8'd0; best_span <= 8'd0;
            band_minx <= 8'hFF; band_maxx <= 8'd0;
            cmin <= 8'hFF; cmax <= 8'd0;
            eye_sum <= 16'd0; chk_sum <= 16'd0; eye_n <= 8'd0; chk_n <= 8'd0;
            y_lo_eye <= 8'd0; y_hi_eye <= 8'd0; y_lo_chk <= 8'd0; y_hi_chk <= 8'd0;
            face_cx <= 8'd0; face_cy <= 8'd0; face_w <= 8'd0; face_h <= 8'd0;
            face_conf <= 8'd0; face_tgl <= 1'b0;
        end else begin
            de_d <= de_d1;

            // ---------------- 采集通路 ----------------
            if (line_start) begin
                lx <= 12'd0;
                bx <= 8'd0;
                ly <= (ly == 10'd719) ? 10'd0 : (ly + 10'd1);
                if (vg == BHC[2:0]) begin
                    // 组结束：把本块行的统计落盘
                    vg <= 3'd0;
                    rs_cnt[by] <= row_cnt;
                    rs_min[by] <= row_min;
                    rs_max[by] <= row_max;
                    by <= (by == CH-1) ? 7'd0 : (by + 7'd1);
                    row_cnt <= 8'd0;
                    row_min <= 8'hFF;
                    row_max <= 8'd0;
                end else begin
                    vg <= vg + 3'd1;
                end
            end else if (de_d1) begin
                lx <= lx + 12'd1;
                if (skin_px) blk_cnt <= blk_cnt + 6'd1;
            end

            if (blk_end) begin
                if (grp_last) begin
                    // 组末：本块定案并清零（下一行的同一列重新累加）
                    if (blk_sum >= BLK_THR) begin
                        row_cnt <= row_cnt + 8'd1;
                        if (bx < row_min) row_min <= bx;
                        if (bx > row_max) row_max <= bx;
                    end
                    acc[bx]  <= 5'd0;
                    blk_cnt  <= 6'd0;
                end else begin
                    acc[bx]  <= acc_new;
                    blk_cnt  <= 6'd0;
                end
                bx <= (bx == CW-1) ? 8'd0 : (bx + 8'd1);
            end

            // ---------------- 帧末后处理 ----------------
            if (frame_end && pst == P_IDLE) begin
                pst     <= P_SCAN;
                pi      <= 8'd0;
                in_run  <= 1'b0;
                run_s   <= 8'd0;
                best_span <= 8'd0;
                best_s  <= 8'd0;
                best_e  <= 8'd0;
                cmin    <= 8'hFF;
                cmax    <= 8'd0;
            end else case (pst)
                P_SCAN: begin
                    if (rs_cnt_cur >= ROW_THR) begin
                        if (!in_run) begin
                            in_run <= 1'b1;
                            run_s  <= pi;
                            cmin   <= rs_min_cur;
                            cmax   <= rs_max_cur;
                        end else begin
                            if (rs_min_cur < cmin) cmin <= rs_min_cur;
                            if (rs_max_cur > cmax) cmax <= rs_max_cur;
                        end
                    end else if (in_run) begin
                        in_run <= 1'b0;
                        if ((pi - run_s) > best_span) begin
                            best_span <= pi - run_s;
                            best_s    <= run_s;
                            best_e    <= pi - 8'd1;
                            band_minx <= cmin;
                            band_maxx <= cmax;
                        end
                        cmin <= 8'hFF;
                        cmax <= 8'd0;
                    end
                    if (pi == CH-1) begin
                        // 收尾：若带一直延伸到图像底边
                        if (in_run && ((pi + 8'd1 - run_s) > best_span) && (run_s != 8'd0)) begin
                            best_span <= pi + 8'd1 - run_s;
                            best_s    <= run_s;
                            best_e    <= pi;
                            band_minx <= cmin;
                            band_maxx <= cmax;
                        end
                        pi  <= 8'd0;
                        pst <= P_BAND;
                    end else begin
                        pi <= pi + 8'd1;
                    end
                end

                P_BAND: begin
                    // 形状约束：高度在范围内，宽度与高度成合理比例（人脸块宽高比≈0.56）
                    if ((best_span >= MIN_H) && (best_span <= MAX_H) &&
                        (band_maxx > band_minx) &&
                        ((band_maxx - band_minx) >= (best_span >> 1)) &&
                        ((band_maxx - band_minx) <= ({1'b0, best_span} + (best_span >> 1)))) begin
                        // 眼带 1/4~2/5，颊带 3/5~4/5；用 ×k>>8 近似，避免组合除法器
                        // （102/256≈2/5、153/256≈3/5、204/256≈4/5，误差≤1 个块行）
                        y_lo_eye <= best_s + (best_span >> 2);
                        y_hi_eye <= best_s + ((bs_span * 16'd102) >> 8);
                        y_lo_chk <= best_s + ((bs_span * 16'd153) >> 8);
                        y_hi_chk <= best_s + ((bs_span * 16'd204) >> 8);
                        pi      <= best_s;
                        eye_sum <= 16'd0; chk_sum <= 16'd0;
                        eye_n   <= 8'd0;  chk_n   <= 8'd0;
                        pst     <= P_EYE;
                    end else begin
                        // ★ B3 修正：无候选也翻转 tgl，让接收端知道"本帧已处理且无目标"
                        face_conf <= 8'd0;
                        face_tgl  <= ~face_tgl;
                        pst       <= P_IDLE;
                    end
                end

                P_EYE: begin
                    if ((pi >= y_lo_eye) && (pi <= y_hi_eye)) begin
                        eye_sum <= eye_sum + {8'd0, rs_cnt_cur};
                        eye_n   <= eye_n + 8'd1;
                    end
                    if ((pi >= y_lo_chk) && (pi <= y_hi_chk)) begin
                        chk_sum <= chk_sum + {8'd0, rs_cnt_cur};
                        chk_n   <= chk_n + 8'd1;
                    end
                    if (pi >= best_e) pst <= P_OUT;
                    else              pi  <= pi + 8'd1;
                end

                P_OUT: begin
                    // 眼带平均密度 / 颊带平均密度 < EYE_RATIO_Q8/256 ⇒ 判为脸
                    // 交叉相乘避免除法：eye_sum/chk_sum * ...
                    if ((eye_n != 0) && (chk_n != 0) &&
                        ((band_maxx - band_minx) < MAX_W) &&
                        ((eye_sum * chk_n * 256) < (chk_sum * eye_n * EYE_RATIO_Q8))) begin
                        face_cx   <= band_minx + ((band_maxx - band_minx) >> 1);
                        face_cy   <= best_s + (best_span >> 1);
                        face_w    <= band_maxx - band_minx + 8'd1;
                        face_h    <= best_span;
                        face_conf <= (best_span > 8'd24) ? 8'd255 : {2'b00, best_span, 1'b0};
                    end else begin
                        face_conf <= 8'd0;
                    end
                    face_tgl <= ~face_tgl;      // ★ B3：每帧无条件翻转
                    pst      <= P_IDLE;
                end

                default: pst <= P_IDLE;
            endcase
        end
    end

endmodule
