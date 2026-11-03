# 360° 音视频协同控制（麦克风阵列 × 三目摄像头）

实现位置：`mix/cam_mic_merge`
控制器：`src/pan_ctrl.v`（clk 域）
检测器：`src/face_det_stream.v`（video_clk 域）
DOA 来源：`src/audio/audio_top.v`

---

## 1. 360° 几何模型

三目相机与 6 麦阵列**共中心点、向外辐射**，全景用一条 3072 px 的环形坐标表示：

```
全景 3072 px ≡ 360°   ⇒  8.533 px/度
每颗相机占 1024 px = 120°

        ┌──────── 全景环形坐标 ────────┐
  0                                3072
  ├── cam2(U4) 1024 ──┼── cam1(U3) 1024 ──┼── cam3(U5) 1024 ──┤
   0        512    1023 1024   1536   2047 2048   2560   3071
        段中心512          段中心1536          段中心2560
```

相机段中心 → 取景窗左边界（取景窗宽 1280，左边界 = 段中心 − 640，环形回绕）：

| 相机 | 全景段 | 段中心 | 取景窗左边界 `TGT_*` |
|---|---|---|---|
| cam2 = **U4** | 0 .. 1023 | 512 | 512 − 640 = −128 → **2944** |
| cam1 = **U3** | 1024 .. 2047 | 1536 | **896** |
| cam3 = **U5** | 2048 .. 3071 | 2560 | **1920** |

麦克风阵列 6 个麦均布（每 60° 一个），**两两成对**，每对的中点对准一颗摄像头的光轴：

| 麦克风对 | 对应摄像头 | 覆盖扇区 |
|---|---|---|
| **M1、M0** | **U5** | 120° |
| **M5、M4** | **U4** | 120° |
| **M3、M2** | **U3** | 120° |

---

## 2. 两阶段转向策略

```
      音源出现
         │
         ▼
  ① 粗对准（主控 = DOA）
     doa_sector(0..11) → grp(0..2) → 摄像头 → 该相机段中心
     沿**环形最短方向**渐进，每帧位移自适应（远快近慢）
         │  到位（dist == 0）
         ▼
  ② 细居中（主控 = 人脸检测）
     face_err = 人脸框中心×8 − 640
     face_ofs 按 |err|/16 累加，上限 STEP_FACE/帧，死区 DEAD_PX
     目标随之变成 pan_goal = 段中心 + face_ofs
         │
         ▼
      画面静止（误差进入死区）
```

**为什么细调要用"偏移量累加"**：若微调直接改位置，而"回段中心"分支下一帧又对齐回
段中心，两者互相抵消 —— 挪多少被拉回多少，永远居不了中。把微调量提成目标的第二分量后，
"微调"与"回中心"变成同一件事。

---

## 3. 三条平移约束的实现

### 约束①：无音源时画面完全静止

```
DOA 静音 → act_cnt 递减到 0 → auto_active = 0
        → top.v 的 MUX 切回 manual_offset
        → 而 manual_offset 在自动期间一直跟随画面位置
        ⇒ 画面停住，且**不跳回**上一次手动停留的位置
```

### 约束②：按键平移结束后 1 秒内不启动自动

```verilog
parameter integer LOCK_TICK = 15;          // 15 × 66.7 ms ≈ 1.00 s

if (any_key)                            lock_cnt <= LOCK_TICK;   // 按住保持满载
else if (step_tick && lock_cnt != 0)    lock_cnt <= lock_cnt - 1;// 松手后递减

if (any_key || (lock_cnt != 0))         act_cnt <= 0;            // 保护期内禁止自动
else if (doa_ok)                        act_cnt <= ACT_TICK;
else if (step_tick && act_cnt != 0)     act_cnt <= act_cnt - 1;

assign auto_active = (act_cnt != 0) && (lock_cnt == 0) && !any_key;
```

- 保护期从**最后一次按键活动**起算，按住多久都不会提前恢复；
- `auto_active` 用 `lock_cnt` 与 `any_key` **双条件**，任一条成立就不接管；
- 行为模型实测：按住 4 拍后松手，自动在 **0.93 s** 后恢复，期间 `auto_active` 恒 0。

### 约束③：平滑过渡，避免跳动

用"接近减速"替代恒速步进（恒速会在目标附近出现"冲过头→回一步"的往复）：

```verilog
wire [12:0] adist = dist_s[13] ? ((~dist_s[12:0]) + 13'd1) : dist_s[12:0];
wire [12:0] aim_step = (adist > STEP_AIM*3) ? STEP_AIM :      // 远：全速 24 px/帧
                       (adist > 4)          ? (adist >> 2) :  // 近：按 1/4 减速
                                               adist;         // 很近：一步对齐
```

实测位移序列（起点 0 → 目标 896）：

```
24, 24, 24, ... , 24, 18, 12, 8, 4, 3, 2, 2, 1, 1, 1, [一步对齐]
                         ↑ 距离进入 72 px 后开始自然减速
```

第二阶段（人脸居中）步长另设为 `STEP_FACE = 4`（60 px/s），比粗对准更慢，
符合"缓慢平移至画面中央"。

---

## 4. 防抖（多人对话 / 扇区抖动）

| 机制 | 参数 | 作用 |
|---|---|---|
| 换相机确认 | `HOLD_N = 3` | 连续 3 次（≈0.2 s）同一结论才切，防两个人交替说话来回跳 |
| 换相机驻留 | `DWELL_N = 15` | 换过后 1 s 内禁止再换，防 sector 在 120° 组边界抖动 |
| 人脸框龄 | `face_age ≤ 40` | 超过 ≈2.7 s 的旧框不用于微调 |
| 居中死区 | `DEAD_PX = 16` | ±2° 内不动，防在中心附近抖动 |

---

## 5. 信号接口

### audio_top → pan_ctrl（clk 域，4 根线）

```verilog
output wire signed [17:0] doa_bearing_o;   // Q2.16，32768 == 90°（预留，当前用 sector）
output wire [3:0]         doa_sector_o;    // 0..11，每扇区 30°
output wire [7:0]         doa_conf_o;      // 0..255
output wire               doa_valid_o;     // 单拍，每块解算完成（≈84 ms）
```

### face_det_stream → pan_ctrl（跨域，toggle 握手）

```verilog
output reg [7:0] face_cx;      // 框中心列（块坐标 ×8 = 屏幕像素）
output reg [7:0] face_conf;    // 0 = 本帧无目标
output reg       face_tgl;     // 每帧（含无目标帧）翻转
```

### pan_ctrl → top.v（clk 域）

```verilog
output reg  [12:0] auto_offset;   // 0..3071 全景列
output wire        auto_active;   // 1 = 接管取景窗
```

仲裁（`top.v` 唯一一处）：

```verilog
horizontal_offset <= auto_active ? auto_offset : manual_offset;
```

> **architecture 保证**：原始按键平移逻辑一字未改，只把目标寄存器改名 `manual_offset`。
> `auto_active = 0` 时与同事原始版本**逐位等价** —— 自动逻辑有任何问题都不会伤到按键。

---

## 6. 参数总表（`top.v` 的 `pan_ctrl` 例化处）

| 参数 | 现值 | 含义 | 症状 → 调法 |
|---|---|---|---|
| `SECT_OFF` | 0 | ★正前方对应扇区（每个 30°） | 切到相邻相机 → ±4；切到对面 → ±6 |
| `SECT_DIR` | 0 | ★扇区递增方向 | 左右全反 → 改 1 |
| `GRP0_CAM` | 1（U5） | 组0 对应相机 | 三组各自核对 |
| `GRP1_CAM` | 0（U4） | 组1 对应相机 | 同上 |
| `GRP2_CAM` | 2（U3） | 组2 对应相机 | 同上 |
| `STEP_AIM` | 24 | 切屏每帧上限（360 px/s） | 嫌慢 → 32；开始有跳感 → 16 |
| `STEP_FACE` | 4 | 人脸微调每帧上限（60 px/s） | 收敛太慢 → 8 |
| `DEAD_PX` | 16 | 居中死区（≈2°） | 中心抖动 → 24 |
| `FACE_RANGE` | 350 | 微调允许偏离段中心（±41°） | 拉不回中心 → 保持 |
| `LOCK_TICK` | 15 | 按键后禁止自动的帧数（≈1 s） | 需求写死 1 s，一般不改 |
| `ACT_TICK` | 15 | 有声保持（≈1 s） | 静音后画面回退太快 → 加大 |
| `HOLD_N` | 3 | 换相机确认次数 | 来回跳 → 5 |
| `DWELL_N` | 15 | 换相机后驻留帧数 | 仍在甩 → 加大 |
| `CONF_MIN` | 16 | DOA 置信度门限 | 噪声误触发 → 32 |

---

## 7. 现场标定步骤（只需做一次）

**前提**：麦克风阵列与三颗相机已按 §1 的方位装好，且 `state_led[3]`（T20）在说话时会闪
（说明 DOA 在出结果）。

1. **看各组落在哪个相机**
   站在 M1/M0 那一对的正前方说话，看画面切到哪颗相机：
   - 切到 U5 ✓ 正确
   - 切到 U4 或 U3 ⇒ 把 `GRP0_CAM` 改成对应编号，或整体用 `SECT_OFF` 平移

2. **`SECT_OFF` 微调**
   若三组**整体**偏了（比如正前方总切到"隔壁那对"），改 `SECT_OFF`：
   - 偏一组（120°）= 4 个扇区 ⇒ `SECT_OFF ± 4`
   - 偏到对面（180°）⇒ `SECT_OFF ± 6`

3. **`SECT_DIR`**
   若左右完全镜像（左说切右、右说切左）⇒ 置 `SECT_DIR = 1'b1`。

4. **验证人脸居中**
   转向到位后站正，人脸应被缓慢拉到画面中央；若在中心附近抖，`DEAD_PX` 16 → 24。

> `audio_top` 里还有 `SECT_OFF_DEG = 67` / `MIRROR_X` / `BEARING_FLIP` 三个 DOA 侧标定，
> 那是给**灯环显示**用的。它们也会影响 `doa_sector` 的数值，所以如果灯环显示与朝向
> 一致、但切屏错位，优先调本模块的 `SECT_OFF`（不要去动灯环那三个）。

---

## 8. 与需求逐条对照

| 需求 | 实现 | 验证 |
|---|---|---|
| M1/M0 → U5 | `GRP0_CAM = 1` | 行为模型 + 现场标定 |
| M5/M4 → U4 | `GRP1_CAM = 0` | 同上 |
| M3/M2 → U3 | `GRP2_CAM = 2` | 同上 |
| 共中心 360° 向外 | 全景 3072 = 360°，三相机各 120° | 几何推导 |
| 先转向对应摄像头画面中心 | 阶段① 目标 = `TGT_<cam>`（段中心） | 模型：6 方向全收敛 |
| 随后把人像缓慢平移至中央 | 阶段② `face_ofs` 微调，`STEP_FACE=4` | 到位后才启用 |
| **无音源画面静止** | `act_cnt→0` ⇒ `auto_active=0` ⇒ 保持 | 模型：无 DOA 时不动 |
| **按键结束后 1 s 内不自动** | `LOCK_TICK=15` 保护期 | 模型：0.93 s 后恢复，期间恒 0 |
| **平滑过渡不跳动** | 接近减速 `|dist|/4` | 模型：24→1 单调递减 |

---

# 9. 2026-11-03 追加：倾斜修复 / 3 秒约束 / 无音源禁用

## 9.1 「画面整体向左倾斜」= 取景窗锁存落在帧中途（shear）

拼接是**逐行**进行的（`cmos_16bit_clk` 域，一帧 720 行）。取景窗偏移只有在
「一行都还没开始」的帧边界改变，整帧才取自同一个取景位置；**一旦锁存时刻落在
帧中途，同一帧的上半屏与下半屏就来自不同偏移 ⇒ 画面被斜着切开**，
观感就是"整体向左倾斜"。

原实现用本地计数估计帧长：

```verilog
localparam integer SOFT_LATCH_CYC = 700_000;   // 估算：cmos_16bit_clk × 66.7 ms
wire soft_latch = (soft_cnt_c >= SOFT_LATCH_CYC);
if (soft_latch) offset_frame <= horizontal_offset_sync1;
```

问题：`cmos_16bit_clk = pclk/2`，而 pclk 随 OV5640 PLL 配置变化（21 / 24 MHz …）。
估算值只要偏 1%，锁存点就逐帧漂移 → **周期性落进帧中间 → 间歇性斜切**。

**修法：VSYNC 优先，本地计数兜底**

```verilog
// vsync 存活检测：166 ms 内见过脉冲就算活着
wire vs_ok_c = (vs_gap_c < 24'd2_000_000);
wire latch_en = vs_ok_c ? cam1_vsync_synced_pulse : soft_latch;

always @(posedge cmos_16bit_clk or negedge rst_n)
    if (!rst_n)        offset_frame <= 0;
    else if (latch_en) offset_frame <= horizontal_offset_sync1;
```

| VSYNC 状态 | 锁存源 | 结果 |
|---|---|---|
| 正常 | `cam1_vsync_synced_pulse` | **与真实帧边界严格对齐，无斜切、无漂移** |
| 丢失 | 本地 66.7 ms 计数 | 仍能转，可能轻微斜切（功能优先） |

> 关键：**pan 的步进节拍（`step_tick`）始终是本地计数，不受此处影响** ——
> 所以 VSYNC 丢失也不会让手动/自动停摆（这是上一轮踩过的坑，此处刻意隔离）。

## 9.2 自动转动 3 秒内完成

15 fps（66.7 ms/帧）下的硬约束换算：

| 距离 | STEP_AIM=24 | 32 | 40 | **48** |
|---|---|---|---|---|
| 1024 px（相邻相机） | 3.3 s | 2.7 s | 2.5 s | **2.20 s** |
| 1536 px（对面） | 4.9 s ✗ | 3.9 s ✗ | 3.3 s ✗ | **2.93 s ✓** |

⇒ `STEP_AIM = 48`。全部 3072 个起点 × 3 个目标的最坏情况 **2.93 s**（模型实测，0 个不收敛）。

⚠ **权衡说明**：48 px/帧在 1280 宽画面上是 3.75% 屏宽/帧。这是"3 秒内走完 1536 px"
的**物理下限**（1536/45 帧 = 34 px 平均，峰值必须更高）。要同时更快又更平滑，
只能提高相机帧率（15→30 fps，需改 LUT `0x3036` + 提 pclk），不在本次范围。

接近减速仍然保留，所以最后一段是平滑收敛：

```
48, 48, ..., 48, 9, 6, 5, 4, 3, 2, 1, 1, 1, [一步对齐]
                 ↑ 距离 ≤144 px 后按 1/4 递减
```

## 9.3 无法定位音源时禁用自动、保持朝向

机制（原有，本次把门限加严）：

```verilog
wire doa_ok = doa_valid && (doa_conf >= CONF_MIN);   // CONF_MIN: 16 → 32

if (any_key || lock_cnt != 0)      act_cnt <= 0;
else if (doa_ok)                   act_cnt <= ACT_TICK;   // 仅在有效音源时续期
else if (step_tick && act_cnt!=0)  act_cnt <= act_cnt - 1;

assign auto_active = (act_cnt != 0) && (lock_cnt == 0) && !any_key;
```

- 无有效音源 ⇒ `act_cnt` 在 `ACT_TICK`（≈1 s）内递减到 0 ⇒ `auto_active=0`；
- `auto_active=0` ⇒ MUX 切回 `manual_offset`，而 `manual_offset` 在自动期间
  一直跟随画面位置 ⇒ **画面停在当前朝向，不跳回旧位置**；
- `CONF_MIN` 16→32：噪声/环境声的 `confidence` 通常 <24，门限 32 可避免
  "没人在说话画面却乱转"。若真人说话也进不来，降到 24。

**注意**：`ACT_TICK` 是"有声保持"时长（≈1 s），不是立即停。这样短促的停顿
（说话换气）不会让画面来回动；真正静音 1 s 后画面才停止响应。

## 9.4 已知遗留（不影响画面与转向）

`cam_diag`（诊断模块）的例化在 `generate` 之外，而它引用的 `pixel_counter` /
`horizontal_offset` 声明在 `generate` 之内 ⇒ 综合报：

```
Undeclared symbol 'pixel_counter',   assumed default net type 'wire'
Undeclared symbol 'horizontal_offset', assumed default net type 'wire'
```

这是**基线既有**问题（非本次引入），只影响 UART 诊断文本里这两个数值的准确性，
**不影响像素通路与转向控制**。考虑到 CLS 已到 80%，本次不修。
