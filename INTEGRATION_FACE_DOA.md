# 人脸检测 × DOA 联动转向：实现与集成说明

工程：`D:\fpga1\mix\cam_mic_merge`（三目拼接 + 7 麦 DOA 的合并工程）
基线：同事原始版本（git v1.1.18，位流 `0xCF47`）
本次新增：`face_det_stream.v`、`pan_ctrl.v`；改动 `top.v`、`audio/audio_top.v`、`cam_mic_merge.sdc`、`cam_mic.gprj`

---

## 1. 设计前提：只用旁路，不替换已验证通路

上一轮集成失败的教训是：把已工作的按键平移逻辑**整段替换**成新模块驱动，
结果新模块任何一处不成立，手动与自动就同时失效，而画面上看不出原因。

这次的结构保证：

```verilog
// top.v：原始按键逻辑一字不改，只把目标寄存器改名 manual_offset
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) ... 
    else if (auto_active) begin           // ← 唯一新增的分支
        manual_offset <= horizontal_offset;   // 自动期间跟随，保证切回无跳变
    end else begin
        ... 同事原始代码，完全不变 ...
    end
end

// 唯一的仲裁点
horizontal_offset <= auto_active ? auto_offset : manual_offset;
```

**推论**：只要 `auto_active = 0`（无 DOA、无按键以外的任何情况），
系统行为与原始版本**逐位等价**，按键一定可用。

---

## 2. 数据流

```
   ┌──────────── audio_top（clk 域 50 MHz）────────────┐
   │ 7 麦 I2S → mic_capture → subband4 → tdoa → doa_solver │
   │                          ↓                        │
   │              bearing / sector / confidence / done │
   └──────────────────────┬────────────────────────────┘
                          │ 新增 4 个输出端口
                          ▼
   ┌──────────────────────────────────────────────┐
   │            pan_ctrl（clk 域）                 │
   │  ① 扇区 → 相机（HOLD_N 次确认）                │
   │  ② 环形最短路径渐进到该相机段中心              │
   │  ③ 到位后人脸微调：目标 = 段中心 + face_ofs    │
   │  ④ 按键/静音 → auto_active=0                  │
   └──────────────────────┬───────────────────────┘
                          │ auto_offset / auto_active
                          ▼
              ┌───────────────────────┐
              │  MUX（top.v 唯一仲裁） │◄── manual_offset（原始按键逻辑）
              └───────────┬───────────┘
                          ▼ horizontal_offset
              同步 → offset_frame（本地 66.7ms 帧锁存）
                          ▼
              拼接流水线（cmos_16bit_clk，逐行）
                          ▼
              DDR3 三帧缓冲 → TMDS 720P → HDMI
                          │
                          ▼ 显示侧像素流 off0_syn_*
   ┌──────────────────────────────────────────────┐
   │       face_det_stream（video_clk 域）         │
   │  肤色分割 → 8×6 块统计 → 行投影定带 → 眼带判据 │
   └──────────────────────┬───────────────────────┘
                          │ face_cx/cy/w/h/conf + face_tgl（toggle 跨域）
                          └──────────────► 回到 pan_ctrl
```

检测器吃的是**显示侧**像素流，与观众看到的完全同源 ⇒ 输出的块坐标 ×8 就是屏幕像素，
不需要任何补偿（也为将来加 OSD 画框省掉换算）。

---

## 3. 端口级接口

### 3.1 `audio_top`（新增 4 个输出，clk 域）

```verilog
output wire signed [17:0] doa_bearing_o;   // Q2.16，32768 == 90°
output wire [3:0]         doa_sector_o;    // 0..11，每扇区 30°
output wire [7:0]         doa_conf_o;      // 0..255，越大越可信
output wire               doa_valid_o;     // 单拍，每块解算完成（≈84 ms）
```

> `assign` 必须写在 `u_doa` 例化**之后**：`bearing/sector/confidence/doa_done` 是
> generate 作用域里的局部线网，写到模块末尾会报 `not declared`。

### 3.2 `face_det_stream`（video_clk 域）

```verilog
// 输入：显示侧像素流
input  wire        px_de;        // off0_syn_de
input  wire [15:0] px_data;      // RGB565
input  wire        vs;           // syn_off0_vs（实际未用于帧判定，见 §3.4）

// 输出：块坐标（0..159 / 0..119）
output reg  [7:0]  face_cx;      // 框中心列 → 像素 = ×8
output reg  [7:0]  face_cy;      // 框中心行 → 像素 = ×6
output reg  [7:0]  face_w;       // 框宽（块数）
output reg  [7:0]  face_h;       // 框高（块行数）
output reg  [7:0]  face_conf;    // 0 = 本帧无目标
output reg         face_tgl;     // 每帧结束翻转（含"无目标"帧）
```

### 3.3 `pan_ctrl`（clk 域）

```verilog
input  wire               step_tick;      // 每帧一拍（本地 66.7 ms 计数）
input  wire signed [17:0] doa_bearing;    // 预留，当前未使用
input  wire [3:0]         doa_sector;
input  wire [7:0]         doa_conf;
input  wire               doa_valid;
input  wire [7:0]         fc_x, fc_conf;  // 人脸（clk 域已同步）
input  wire               fc_tgl;
input  wire               any_key;        // 任一按键按下（高有效，已消抖）
input  wire [12:0]        manual_offset;
output reg  [12:0]        auto_offset;    // 0..3071 全景列
output wire               auto_active;
output wire [1:0]         dbg_cam;        // 0=U4 1=U5 2=U3
output wire [3:0]         dbg_sect;       // 标定后的扇区值
```

### 3.4 帧边界为什么不用 `vs`

`top.v:901` 把 `I_vout0_vs_n` 接的是 `~syn_off0_vs`，极性在 IP 两侧被取反过一次；
靠极性推断"上升沿=帧首"不稳。改用 **`de` 连续为低 64 拍** 判定消隐期，
任何极性下都成立，且 `de_gap==63` 天然是单拍脉冲。

---

## 4. 控制律

### 4.1 主控：DOA → 相机

```
sec  = (sector + SECT_OFF) mod 12        // 标定偏移
sec  = SECT_DIR ? (11 - sec) : sec       // 方向标定
grp  = sec[3:2]                          // 每 4 扇区一组 = 120°
cam  = grp0→U4 / grp1→U5 / grp2→U3       // 按硬件方向映射
```

相机取景窗左边界（全景 3072 px = 360°，每颗占 1024 px，取景窗宽 1280）：

| 相机 | 全景段 | 段中心 | 取景窗左边界 |
|---|---|---|---|
| cam2 = U4 | 0..1023 | 512 | 512−640 = −128 → 回绕 **2944** |
| cam1 = U3 | 1024..2047 | 1536 | **896** |
| cam3 = U5 | 2048..3071 | 2560 | **1920** |

换相机需连续 `HOLD_N=3` 次同一结论（≈0.2 s），防止两个人交替说话时画面来回跳。
转向沿**环形最短方向**，每帧 `STEP_AIM=24 px`（66.7 ms/步 ⇒ 360 px/s，切一颗相机 1024 px ≈ 2.8 s）。

### 4.2 微调：人脸居中

```
目标 = 相机段中心 + face_ofs
f_err = face_cx×8 − 640                  // 偏右为正
face_ofs += clamp(|f_err|/16, 1..STEP_FACE)   // 带符号，死区 DEAD_PX
face_ofs 限幅 ±FACE_RANGE
```

**为什么用"偏移量累加"而不是直接改位置**：若微调直接改 `auto_offset`，而"回段中心"分支
下一帧又把位置对齐回段中心，两者互相抵消 —— 挪多少被拉回多少，永远到不了正中间。
把微调量提成目标的第二分量后，"微调"与"回中心"变成同一件事。

死区 `DEAD_PX=16`（≈2°）：太小会在中心附近抖，太大会偏。步长按误差自适应：
偏 240 px 时 8 px/帧（≈1 s 收敛），进入 ±40 px 自动减速，不会过冲。

### 4.3 优先级与回退

| 条件 | 行为 |
|---|---|
| 任一按键按下 | `act_cnt` 当拍清零 ⇒ `auto_active=0` **立即**生效，走 `manual_offset` |
| 自动期间按按键 | 无跳变（`manual_offset` 一直在跟随 `horizontal_offset`） |
| DOA 有效（`conf ≥ CONF_MIN`） | `act_cnt` 续期 `ACT_TICK=15`（≈1 s） |
| 静音超 1 s | `auto_active=0`，画面**停在原地**（无按键时 MUX 保持 manual_offset） |

### 4.4 已知的三个检测器缺陷（本次重写修掉）

| # | 缺陷 | 后果 |
|---|---|---|
| B1 | 块累加器读成了"上一块"的值（`acc_rd` 只在 `blk_end` 更新） | 累加跨块串扰，**所有块统计都是错的 ⇒ 永远检不出** |
| B2 | 行统计数组打了一拍，扫描索引与数据错位一行 | 段边界整体偏一行，最后一行永远扫不到 |
| B3 | `face_tgl` 只在"检出"时翻转 | 没检出时接收端等不到边沿，**一直沿用上一帧的旧框** |

修法：B1 改组合读 `acc[bx]`；B2 改组合读 `rs_cnt[pi]`；B3 改每帧无条件翻转。

---

## 5. 约束更新

### 5.1 时序（`src/cam_mic_merge.sdc`，新增 1 条）

```tcl
set_false_path -from [get_clocks {clk_74_25}] -to [get_clocks {clk}]
```

人脸结果跨域是**握手式 CDC**（数据稳定整帧 66.7 ms + toggle 两级同步），没有单周期要求。
用 `set_false_path` 而不是 `set_max_delay` 的原因：

- `-datapath_only` 在 Gowin V1.9.11.03 **不支持**（`TA2000` 语法错，构建 EXIT=1）；
- 不加 `-datapath_only` 的 `set_max_delay` 对"同一 `set_clock_groups` 组内的派生时钟"实测无效。

本设计里 `clk_74_25 → clk` 只有这一处跨域，不会掩盖其他真实路径。

### 5.2 引脚：**零改动**

`cam_mic_merge.cst` 一个字都没改，I/O 仍是 **143/297**。原因：

1. 检测器输入是**内部像素流**（`off0_syn_*`），不是引脚；
2. "转向"是**数字云台**（改全景取景窗），没有电机/舵机引脚；
3. 按键沿用已有的 `key_left`(Y12) / `key_right`(AB13)。

---

## 6. 资源与延迟

| 项 | 数值 |
|---|---|
| 新增逻辑 | `face_det_stream` ≈1.5K LUT、0 DSP、0 乘法器 |
|  | `pan_ctrl` ≈300 LUT、1 个小比较器链 |
| 新增存储 | 块累加器 160×5bit + 行统计 3×120×8bit（分布式 RAM） |
| 端到端延迟 | 声 → DOA 解算 ≈84 ms；转向一步 66.7 ms；切一颗相机 ≈2.8 s |
| 人脸居中收敛 | 偏 240 px 时约 1 s（8 px/帧），死区 ±16 px |

**没有画框（OSD）**：之前实测该设计 CLS 已到 78%，给像素输出末级加 24 个寄存器
就会 `PR0004: 22 unrouted nets` 布线失败；纯组合实现又会破坏 DVI 输出。
本版只输出坐标，不叠加到画面。要画框须先腾资源（把 `rs_*` 显式搬 BSRAM，
现 BSRAM 54/118 还有余量）。

---

## 7. 上板验证顺序

| 步 | 操作 | 预期 | 不符时 |
|---|---|---|---|
| 1 | **按住左右键** | 画面平移 ≈960 px/s（原始手感） | 不动 ⇒ 我的改动破坏了按键，立刻回滚 |
| 2 | **对着板子说话** | 约 2~3 s 内画面转到你所在的相机 | 不转 ⇒ DOA 没输出（查麦克风接线/`CONF_MIN`） |
| 3 | 转到位后**露出正脸** | 人脸被缓慢拉到画面中央，±16 px 内停住 | 不居中 ⇒ 调 `FACE_RANGE`/`DEAD_PX`/`STEP_FACE` |
| 4 | 换方向说话 | 约 0.2 s 确认后切另一颗相机 | 转到错误的相机 ⇒ 标定 `SECT_OFF`/`SECT_DIR` |
| 5 | 说话中途按按键 | 立即转手动，且不再自动 | — |

### 标定方法

- **`SECT_OFF`**：站在你定义的"正前方"说话，看画面切到哪颗相机。
  若切到相邻的 → 改 ±4；切到对面 → 改 ±6。
- **`SECT_DIR`**：若所有方向都切反（左↔右），置 `1'b1`。

---

## 8. 可调参数总表（都在 `top.v` 的例化处）

| 参数 | 现值 | 作用 | 症状 → 调法 |
|---|---|---|---|
| `SECT_OFF` | 0 | 正前方扇区标定 | 切错相机 → ±4 / ±6 |
| `SECT_DIR` | 0 | 扇区方向 | 左右全反 → 1 |
| `STEP_AIM` | 24 | 切屏速度 | 嫌慢 → 32（再大开始有跳感） |
| `STEP_FACE` | 8 | 人脸微调上限 | 收敛慢 → 12；中心晃 → 4 |
| `DEAD_PX` | 16 | 居中死区 | 抖 → 24；不够准 → 10 |
| `FACE_RANGE` | 350 | 微调允许偏离段中心 | 拉不回来 → 保持；画面被误检拉跑 → 220 |
| `CONF_MIN` | 16 | DOA 置信度门限 | 噪声误触发 → 32；不响应 → 8 |
| `FACE_CONF_MIN` | 16 | 人脸置信度门限 | 误跟背景 → 32 |
| `HOLD_N` | 3 | 换相机确认次数 | 来回跳 → 5 |
| `BLK_THR` / `ROW_THR` | 6 / 8 | 检测灵敏度 | 检不出 → 4 / 6；误检 → 8 / 10 |
| `MAX_W` | 128 | 拒绝大面积肤色背景 | 框贴墙 → 96 |
| `EYE_RATIO_Q8` | 205 | 眼带密度比门限 | 误检 → 170；漏检 → 230 |

---

## 9. 仍然存在的限制

| 限制 | 原因 | 影响 |
|---|---|---|
| 检测器精度有限 | 肤色 + 眼带判据，非 Haar/CNN（本板无 NPU、时钟网络已满、离线取不到级联表） | 侧脸 >45°、强逆光、帽子会漏检；木质/纸箱背景可能误检 |
| 接缝两侧时间不同步 | 三颗 OV5640 自由运行、无 genlock | 转动/有人走动时接缝处有错位感 |
| 每帧位移颗粒 | 拼接逐行进行，偏移只能在帧边界变 | 转动颗粒 = 每帧位移；要更顺需提相机帧率（15→30 fps） |
| 不能画框 | CLS 76% 已到布线上限 | 需先腾资源 |
| "云台"是数字的 | 板上无物理云台 | 相机不动，改的是全景取景窗 |

---

# 10. 修复记录（2026-11-02 23:45）：说话不转的真正原因

上板结果：**① 按键平移 ✓，② 自动转向 ✗，③ 人脸居中 ✗，④ 换相机 ✗**。
"有的生效、有的不生效"说明控件旁路架构是对的，问题只落在 DOA 这条链上。

## 10.1 根因：`doa_*` 声明落在 generate 的内/外之间

`top.v` 的结构：

```
top.v:437   wire signed [17:0] doa_bearing;   ← 现在在这里（正确）
top.v:526   generate if(USE_TPG == "false") begin
   :1045        pan_ctrl  u_pan_ctrl(... .doa_sector(doa_sector) ...)   ← generate 内
   :1040    endgenerate
top.v:1362  audio_top u_audio(... .doa_sector_o(doa_sector) ...)        ← generate 外
```

两个例化位置都要看到同一组 `doa_*` 线。三种写法的实测结果：

| 写法 | 结果 |
|---|---|
| 声明在 generate **内**（原版） | `audio_top` 侧变成模块级**隐式 1 位网**；`pan_ctrl` 侧那组线悬空 ⇒ `doa_ok` 恒 0 ⇒ **永不转向** |
| 声明在模块级但**在 generate 之后** | Verilog 要求先声明后使用，Gowin 仍按 1 位处理 |
| **声明在模块级且在 generate 之前** | ✓ 两侧看到同一组 18/4/8/1 位线 |

## 10.2 ★ 这是静默错误：综合不报错，只报 warning

```
Undeclared symbol 'doa_bearing', assumed default net type 'wire'("top.v":1032)
Undeclared symbol 'doa_sector',  assumed default net type 'wire'("top.v":1033)
Undeclared symbol 'doa_conf',    assumed default net type 'wire'("top.v":1034)
Undeclared symbol 'doa_valid',   assumed default net type 'wire'("top.v":1035)
```

而 `build_syn.tcl → EXIT=0`、`build.tcl → EXIT=0`、TNS 全 0、位流正常生成 ——
**全绿但功能全废**。

> **验收必须查这条**：
> ```bash
> grep -i "Undeclared symbol" impl/gwsynthesis/build_syn.log
> ```
> 不能只看 EXIT 码。基线本身也有同类警告（`cmos_clk`/`aux_clk` 是 1 位时钟，恰好无害；
> `line_buffer_wr_ptr_cam2/3` 在 :1314 被隐式声明，属遗留问题）。

## 10.3 同时加了板载 LED 诊断（避免再盲调）

`state_led[3:1]` 临时改为 DOA 联动指示（原含义为三路相机 I2C 错误）：

| LED | 位置 | 含义 |
|---|---|---|
| `state_led[4]` | U20 | cam2/cam3 FIFO 溢出（**保留**） |
| `state_led[3]` | T20 | **DOA 在解算**：`doa_valid` 展宽到 200 ms，会闪 |
| `state_led[2]` | W21 | **DOA 置信度达标**：`doa_conf >= 16`，持续亮 |
| `state_led[1]` | Y18 | **自动接管中**：`auto_active=1` |
| `state_led[0]` | W17 | 系统未就绪（**保留**） |

### 判读表（下次不转时直接看灯）

| 现象 | 结论 | 下一步 |
|---|---|---|
| **三个灯全灭** | DOA 从未解算 | 查麦克风阵列接线（A13/A14/B13/B15/B16/C13/C14/C15 飞线）、`mic_array_bck/ws` 是否有 3.125 MHz |
| **[3] 闪、[2] 灭** | 有解算但置信度不够 | 调小 `pan_ctrl` 的 `CONF_MIN`（现 16）；或说话大点声/靠近阵列 |
| **[2] 亮、[1] 灭** | 逻辑层：`any_key` 被误判为按下 | 查按键引脚电平（Y12/AB13 应上拉为高，按下为低） |
| **[1] 亮但画面不动** | `pan_ctrl → offset` 通路 | 看 `auto_offset` 是否在变（`dbg_cam`/`dbg_sect` 已接出） |
| **[1] 亮且画面在动但转错相机** | 标定问题 | 调 `SECT_OFF` / `SECT_DIR` |

### 要恢复原来的 I2C 指示

`top.v` 里：把 428-430 行三行 `assign state_led[3..1] = i2c_err_cam*` 取消注释，
并删掉 generate 内那三条 `assign state_led[3..1] = <诊断信号>`。
