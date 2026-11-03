//=============================================================================
//  pan_ctrl.v  —  DOA 主控转向 + 人脸微调居中（clk 域）
//
//  【职责边界】本模块**只产生一个候选取景窗位置 auto_offset** 和一个开关
//  auto_active，绝不直接改已工作的按键平移逻辑。top.v 里用一级 MUX 选择：
//
//      horizontal_offset <= auto_active ? auto_offset : manual_offset;
//
//  这样即使本模块全错，只要 auto_active=0，按键就是原始行为 —— 这是 2026-11-02
//  反复失败后定下的铁律：**新功能只能旁路叠加，不能替换已验证通路**。
//
//  【控制律】
//    · 主控（DOA）：sector(0..11) → 每 4 个扇区一组(120°) → 目标相机
//        grp = ((sector + SECT_OFF) mod 12) >> 2
//        组 → 相机取景窗左边界：grp0→U4、grp1→U5、grp2→U3
//        连续 HOLD_N 次同一结论才切换（防两个人说话时来回跳）
//        沿**环形最短方向**每帧走 STEP_AIM 像素，到位即停
//    · 微调（人脸）：目标 = 相机段中心 + face_ofs
//        face_err = 人脸框中心 - 640（屏幕像素，偏右为正）
//        face_ofs 按 |err|/16 自适应累加（上限 STEP_FACE/帧），死区 DEAD_PX，
//        限幅 ±FACE_RANGE。用"偏移量累加"而不是直接改位置，是为了避免
//        "微调挪一点、回中心分支又拉回一点"互相抵消（这是旧版居不中的根因）。
//    · 手动优先：any_key 立即把 act_cnt 清零 ⇒ auto_active=0 当拍生效。
//    · 静音回退：DOA 有效续期 ACT_TICK×step_tick（≈1 s）；超时后 auto_active=0，
//        画面停在原地（top.v 的 MUX 在无按键时保持），不跳回旧的手动位置。
//
//  【为什么取景窗左边界是这些值】
//    全景 3072 px = 360° ⇒ 每颗相机占 1024 px：
//      cam2(U4) 段 0..1023    中心  512 ⇒ 左边界 512-640 = -128 → 回绕 2944
//      cam1(U3) 段 1024..2047 中心 1536 ⇒ 左边界 896
//      cam3(U5) 段 2048..3071 中心 2560 ⇒ 左边界 1920
//    （取景窗宽 1280，左边界 = 段中心 - 640）
//=============================================================================
`timescale 1ns / 1ps

module pan_ctrl #(
    parameter integer PAN_W        = 3072,
    // 三个相机取景窗左边界（见上）
    parameter integer TGT_U4       = 2944,
    parameter integer TGT_U5       = 1920,
    parameter integer TGT_U3       = 896,
    // 步长与门限
    parameter integer STEP_AIM     = 24,     // 切屏每帧**上限**（66.7ms/步 → 360 px/s，接近时自动减速）
    parameter integer STEP_FACE    = 4,      // 人脸微调每帧上限（60 px/s，要求"缓慢平移"）
    parameter integer DEAD_PX      = 16,     // 居中死区（≈2°）
    parameter integer LOCK_TICK    = 15,     // ★ 按键结束后禁止自动的时长（×66.7ms ≈ 1.0 s，需求②）
    // ★ 组(grp) → 摄像头 的对应表（0=U4 1=U5 2=U3）
    //   用户给定：M1/M0→U5、M5/M4→U4、M3/M2→U3。
    //   grp 由 doa_sector 分组而来，而 sector 的物理零点受 audio_top 的
    //   SECT_OFF_DEG(67)/MIRROR_X/BEARING_FLIP 三个标定影响，所以这张表要配合
    //   本模块的 SECT_OFF/SECT_DIR 一起用；现场按"切错就换"调整。
    parameter integer GRP0_CAM     = 1,      // 组0 → U5
    parameter integer GRP1_CAM     = 0,      // 组1 → U4
    parameter integer GRP2_CAM     = 2,      // 组2 → U3
    parameter integer FACE_RANGE   = 350,    // 相对相机中心的微调范围（±41°）
    parameter integer CONF_MIN     = 16,     // DOA 置信度门限（0..255）
    parameter integer FACE_CONF_MIN= 16,     // 人脸置信度门限
    parameter integer HOLD_N       = 3,      // 换相机需连续确认次数
    parameter integer DWELL_N      = 15,     // 换过相机后至少驻留多少帧才允许再换（×66.7ms ≈1 s）
    parameter integer ACT_TICK     = 15,     // 有声保持时长（×66.7ms ≈ 1 s）
    parameter [3:0]   SECT_OFF     = 4'd0,   // ★标定：正前方说话对应哪个扇区
    parameter         SECT_DIR     = 1'b0    // ★标定：扇区递增是否与全景列同向
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               step_tick,      // 每相机帧一拍（本地 66.7 ms 计数）

    // ---- DOA（audio_top 输出，clk 域）----
    input  wire signed [17:0] doa_bearing,    // Q2.16，未直接使用，留作扩展
    input  wire [3:0]         doa_sector,     // 0..11
    input  wire [7:0]         doa_conf,       // 0..255
    input  wire               doa_valid,      // 单拍

    // ---- 人脸检测结果（视频域经 toggle 同步后的值）----
    input  wire [7:0]         fc_x,           // 框中心列（块坐标，×8 = 像素）
    input  wire [7:0]         fc_conf,
    input  wire               fc_tgl,         // 视频域每帧翻转

    // ---- 手动 / 仲裁 ----
    input  wire               any_key,        // 任一按键按下（高有效，已消抖）
    input  wire [12:0]        manual_offset,  // 当前手动取景窗位置

    // ---- 输出 ----
    output reg  [12:0]        auto_offset,
    output wire               auto_active,    // 1 = 本模块接管取景窗
    output wire [1:0]         dbg_cam,        // 当前相机（0=U4 1=U5 2=U3）
    output wire [3:0]         dbg_sect        // 标定后的扇区值
);

    //==================================================================
    // 1) 人脸结果跨域采样（toggle 握手：数据稳定整帧，边沿处取数）
    //==================================================================
    reg fc_tgl_s0, fc_tgl_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fc_tgl_s0 <= 1'b0;
            fc_tgl_s1 <= 1'b0;
        end else begin
            fc_tgl_s0 <= fc_tgl;
            fc_tgl_s1 <= fc_tgl_s0;
        end
    end
    wire fc_new = fc_tgl_s0 ^ fc_tgl_s1;

    reg [7:0] fx_r, fconf_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fx_r <= 8'd0;
            fconf_r <= 8'd0;
        end else if (fc_new) begin
            fx_r    <= fc_x;
            fconf_r <= fc_conf;
        end
    end

    // 人脸"新鲜度"：每帧 +1，超过 40 帧（≈2.7 s）视为过期
    reg [7:0] face_age;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                              face_age <= 8'hFF;
        else if (fc_new)                         face_age <= 8'd0;
        else if (step_tick && face_age != 8'hFF) face_age <= face_age + 8'd1;
    end
    wire face_ok = (fconf_r >= FACE_CONF_MIN[7:0]) && (face_age <= 8'd40);

    //==================================================================
    // 2) 扇区 → 相机
    //==================================================================
    wire       doa_ok  = doa_valid && (doa_conf >= CONF_MIN[7:0]);

    // 加上标定偏移后对 12 取模
    wire [4:0] sec_sum = {1'b0, doa_sector} + {1'b0, SECT_OFF};
    wire [3:0] sec_raw = (sec_sum >= 5'd12) ? (sec_sum[3:0] - 4'd12) : sec_sum[3:0];
    // 方向标定：反了就镜像
    wire [3:0] sec     = SECT_DIR ? (4'd11 - sec_raw) : sec_raw;
    wire [1:0] grp     = sec[3:2];              // 0..11 >> 2 = 0..2

    // 组 → 摄像头编号 → 取景窗左边界
    wire [1:0] cam_sel = (grp == 2'd0) ? GRP0_CAM[1:0] :
                         (grp == 2'd1) ? GRP1_CAM[1:0] : GRP2_CAM[1:0];
    wire [12:0] tgt_cam = (cam_sel == 2'd0) ? TGT_U4[12:0] :
                          (cam_sel == 2'd1) ? TGT_U5[12:0] : TGT_U3[12:0];

    //==================================================================
    // 3) 相机确认（连续 HOLD_N 次同结论才切换，防抖动）
    //==================================================================
    reg [1:0] cam_cur;
    reg [3:0] sw_cnt;
    reg [7:0] dwell_cnt;                     // 驻留倒计数：换过相机后 DWELL_N 帧内不再换
    reg       cam_sw_pulse;                  // 确认换相机那一拍（单拍，交给下面的块清零 face_ofs）
    reg signed [12:0] face_ofs;              // 相对相机段中心的人脸微调偏移

    assign dbg_cam  = cam_cur;
    assign dbg_sect = sec;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cam_cur      <= 2'd0;
            sw_cnt       <= 4'd0;
            dwell_cnt    <= 8'd0;
            cam_sw_pulse <= 1'b0;
        end else begin
            cam_sw_pulse <= 1'b0;            // 默认单拍

            // 驻留倒计数
            if (step_tick && dwell_cnt != 8'd0) dwell_cnt <= dwell_cnt - 8'd1;

            if (!auto_active) begin
                sw_cnt <= 4'd0;              // 未接管：不积累切换请求
            end else if (step_tick && doa_ok) begin
                if (grp == cam_cur) begin
                    sw_cnt <= 4'd0;
                end else if ((dwell_cnt == 8'd0) &&      // ★ 驻留保护：刚换过就不许再换
                             (sw_cnt >= HOLD_N[3:0] - 4'd1)) begin
                    cam_cur      <= grp;     // 确认切换
                    sw_cnt       <= 4'd0;
                    dwell_cnt    <= DWELL_N[7:0];
                    cam_sw_pulse <= 1'b1;
                end else begin
                    sw_cnt <= sw_cnt + 4'd1;
                end
            end
        end
    end

    //==================================================================
    // 4) 目标 = 相机段中心 + 人脸微调（环形归一）
    //==================================================================
    wire signed [13:0] goal0 = $signed({1'b0, tgt_cam}) + face_ofs;
    wire        [12:0] goal  = (goal0 < 0)                ? (goal0 + PAN_W[13:0]) :
                               (goal0 >= $signed(PAN_W[13:0])) ? (goal0 - PAN_W[13:0]) :
                               goal0[12:0];

    //---------------------------------------------------------------------
    // 环形最短距离：折到 [-PAN_W/2, +PAN_W/2]
    //
    // ★ 2026-11-03 修 BUG：不能先做成 13 位无符号再判符号。
    //   旧写法 `wire [12:0] diff0 = goal - auto_offset;` 在 13 位里做减法，
    //   负差会被回绕成大正数（−4 → 8188），随后的 `diff0 > PAN_W/2` 判断
    //   仍按"正数"理解，于是算出 +5116 而不是 −4 ⇒ **朝反方向一路走下去**，
    //   绕全景一圈约 8.5 s、永远追不上目标 —— 表现就是"一出声音画面就一直
    //   平移，到目标也不停"。
    //   数值实测：goal=896 时 14 个起点中旧实现错 8 个。
    //
    //   正确做法：先在 14 位有符号里求差（范围 −3071..+3071，绝无回绕），
    //   再按 ±PAN_W/2 折到最短方向。
    //---------------------------------------------------------------------
    localparam signed [13:0] HALF_W = PAN_W / 2;
    localparam signed [13:0] FULL_W = PAN_W;

    wire signed [13:0] d_raw  = $signed({1'b0, goal}) - $signed({1'b0, auto_offset});
    wire signed [13:0] dist_s = (d_raw >  HALF_W) ? (d_raw - FULL_W) :
                                (d_raw < -HALF_W) ? (d_raw + FULL_W) : d_raw;

    //==================================================================
    // 4b) 切屏步长：远时全速、近时按比例减速
    //
    //   需求③「平滑过渡，避免画面跳动」。恒速步进在接近目标时会出现
    //   "冲过头 → 回来一步"的往复（尤其在 ±STEP_AIM 边界），肉眼就是抖动。
    //   做法：|dist| > 3×STEP_AIM 时走 STEP_AIM；再近就按 |dist|/4 递减；
    //   到 4 px 内一步对齐。等效一条减速曲线，无需额外状态。
    //==================================================================
    wire [12:0] adist = dist_s[13] ? ((~dist_s[12:0]) + 13'd1) : dist_s[12:0];
    wire [12:0] aim_step = (adist > (STEP_AIM[12:0] * 3)) ? STEP_AIM[12:0] :
                           (adist > 13'd4)                ? (adist >> 2)   :
                                                            adist;

    //==================================================================
    // 5) 人脸误差与自适应步长
    //==================================================================
    wire [11:0]      fx_px = {fx_r, 3'b000};                       // 块列 → 像素
    wire signed [12:0] f_err = $signed({1'b0, fx_px}) - 13'sd640;  // 偏右为正
    wire [11:0]      emag  = f_err[12] ? (~f_err[11:0] + 12'd1) : f_err[11:0];
    wire [11:0]      e_div = emag >> 4;                            // |err|/16
    wire [11:0]      f_step = (e_div >= STEP_FACE[11:0]) ? STEP_FACE[11:0] :
                              (e_div == 12'd0)           ? 12'd1 : e_div;

    // 有符号化，避免与负的 face_ofs 混算时整条表达式退化成无符号
    wire signed [12:0] f_step_s  = {1'b0, f_step};
    localparam signed [12:0] FRANGE_S = FACE_RANGE;
    wire signed [13:0] ofs_p = face_ofs + f_step_s;
    wire signed [13:0] ofs_m = face_ofs - f_step_s;

    //==================================================================
    // 6) 活跃判据 + 按键保护期
    //
    //   三条平移约束的落点：
    //     ① 无音源 → 画面完全静止
    //        act_cnt 归零 ⇒ auto_active=0 ⇒ top.v 的 MUX 切回 manual_offset，
    //        而 manual_offset 在自动期间一直跟随画面位置 ⇒ 画面停住且不跳变。
    //     ② 按键平移结束后 1 秒内不得启动自动
    //        lock_cnt 在"按住期间"保持满载、松手后按 step_tick 递减 LOCK_TICK
    //        （15×66.7ms ≈ 1.0 s）；期间 act_cnt 强制清零，DOA 再有效也不接管。
    //        注意保护期从**最后一次按键活动**起算，按住多久都不会提前恢复。
    //     ③ 平滑 → 见 4b 的 aim_step 与 5 的脸步长
    //==================================================================
    reg [7:0] act_cnt;
    reg [7:0] lock_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_cnt  <= 8'd0;
            lock_cnt <= 8'd0;
        end else begin
            // ---- 保护期倒计数 ----
            if (any_key)                             lock_cnt <= LOCK_TICK[7:0];
            else if (step_tick && lock_cnt != 8'd0)  lock_cnt <= lock_cnt - 8'd1;

            // ---- 活跃计数 ----
            if (any_key || (lock_cnt != 8'd0))       act_cnt <= 8'd0;   // 手动/保护期：禁止自动
            else if (doa_ok)                         act_cnt <= ACT_TICK[7:0];
            else if (step_tick && act_cnt != 8'd0)   act_cnt <= act_cnt - 8'd1;
        end
    end

    // 双保险：auto_active 同时受 lock_cnt 与 any_key 约束
    assign auto_active = (act_cnt != 8'd0) && (lock_cnt == 8'd0) && !any_key;

    //==================================================================
    // 7) 取景窗位置更新
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            auto_offset <= 13'd0;
        end else if (!auto_active) begin
            // 未接管：跟随手动位置，保证接管瞬间无跳变
            auto_offset <= manual_offset;
        end else if (step_tick) begin
            if (dist_s > $signed({1'b0, aim_step})) begin
                // 正向（列号增大方向）—— 越界回绕
                auto_offset <= (auto_offset >= PAN_W[12:0] - aim_step)
                             ? (auto_offset + aim_step - PAN_W[12:0])
                             : (auto_offset + aim_step);
            end else if (dist_s < -$signed({1'b0, aim_step})) begin
                auto_offset <= (auto_offset < aim_step)
                             ? (auto_offset + PAN_W[12:0] - aim_step)
                             : (auto_offset - aim_step);
            end else if (dist_s != 0) begin
                auto_offset <= goal;                 // 收尾对齐，消除残差
            end else if (face_ok) begin
                // 已对准相机：用"偏移量累加"把人拉到画面中心
                //   f_err > 0（脸偏右）⇒ 取景窗右移 ⇒ face_ofs 增大
                if (cam_sw_pulse) begin
                    face_ofs <= 13'sd0;      // 换了相机，旧的人脸偏移作废
                end else if (f_err > $signed(DEAD_PX[12:0]) && (face_ofs < FRANGE_S)) begin
                    face_ofs <= (ofs_p > FRANGE_S) ? FRANGE_S : ofs_p[12:0];
                end else if (f_err < -$signed(DEAD_PX[12:0]) && (face_ofs > -FRANGE_S)) begin
                    face_ofs <= (ofs_m < -FRANGE_S) ? (-FRANGE_S) : ofs_m[12:0];
                end
            end
        end
    end

endmodule
