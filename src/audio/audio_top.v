//-----------------------------------------------------------------------------
// top.v  --  Sipeed 6+1 Mic Array capture + DOA on Tang MEGA 60K
//
//   Signal chain:
//     6+1 mic array --I2S(bck/ws/d[3:0])--> i2s_rx_8slot
//                                        --> mic_capture (map + decimate)
//                  +---------------------+----------------------+
//                  |                                            |
//                  v                                            v
//        mic_uart_framer --> uart_tx --> 'txp' (U15) --> PC   tdoa_engine
//                                                               |
//                                                               v
//                                                          tdoa_filter
//                                                    (per-mic TDOA low-pass)
//                                                               |
//                                                               v
//                                                          doa_solver
//                                                               |
//                                        +----------------------+---------+
//                                        v                                v
//                                 direct ring drive                 leds[3:0]
//                                 (ANY_SND_E energy gate)       (quality lamps)
//                                        |
//                                        v
//                                        sk9822_ring (12 LEDs on the array)
//
//   RAW PCM ALSO GOES OUT THE UART, BUT AS A SAMPLE, NOT A STREAM.
//   The DOA path taps the same captured channels, so the two do not
//   interfere -- but the UART link cannot carry the full frame rate:
//     26 bytes/packet at 115200 8N1  =  11520 B/s  =  443 packets/s
//     array frame rate               =  BCK/64  ~=  48 kHz
//   so only about 1 frame in 108 is transmitted; `dropped` counts the rest
//   and LED[3] latches once any frame has been dropped.  Use the UART for
//   spot checks of the raw waveform, not as a continuous recorder.
//
//   Board: Sipeed Tang MEGA 60K, GW5AT-LV60PG484AC1/I0, Device Version B
//   Clocks:
//     clk      = V22, on-board 50 MHz oscillator (LVCMOS33)
//     rst_n    = AA13, user reset key (LVCMOS15, active low, pulled up)
//
//   Mic array pins: FLYING LEADS at 3.3V (changed 2026-10-14).  The array is
//   no longer plugged into J32; its eight signals are wired by hand to pins
//   in the board's 3.3V SDRAM bank, because that is the level the array side
//   needs.  See audio.cst for the full table and the three caveats (SDRAM
//   pins reused, level compatibility both ways, flying-wire signal integrity).
//     mic_array_bck  A13   (was sdram_dq[7])   MIC_CK  out  3.125 MHz
//     mic_array_ws   A14   (was sdram_dq[6])   MIC_WS  out  48.83 kHz
//     mic_array_d[0] B13   (was sdram_dq[4])   MIC_D0  in
//     mic_array_d[1] B15   (was sdram_dq[14])  MIC_D1  in
//     mic_array_d[2] B16   (was sdram_dq[15])  MIC_D2  in
//     mic_array_d[3] C13   (was sdram_dq[5])   MIC_D3  in
//     mic_led_clk    C14   (was sdram_dq[3])   LED_CK  out  \
//     mic_led_dat    C15   (was sdram_dq[2])   LED_DA  out  / 2-wire, 12 LEDs
//   The old J32 pins (W12/AA9/AA10/AB10/AA11/AB11/W10/Y11, BANK9 @1.5V) are
//   now unused.  The array still needs its own 5V feed.
//
//   本次移植改动 (2026-10-11)
//   -------------------------
//   * 引脚: 2026-10-11 按 J32(FPC-10P) 重绑; 2026-10-14 阵列改为飞线 + 3.3V,
//     8 根信号改绑 A13/A14/B13/B15/B16/C13/C14/C15 (原 SDRAM BANK),
//     与 audio.cst 一一对应; 原 J32 引脚 (BANK9 @1.5V) 全部弃用
//   * FPGA 改为 I2S 主模式: 由 clk/16 产生 MIC_CK(3.125MHz) 与 MIC_WS
//     (48.83kHz) 并从引脚输出。七颗 MSM261S4030H0 是被动从机(SCK/WS 直接
//     来自 J32 脚8/脚7, 模块上没有时钟源), 之前误以为阵列自己出时钟,
//     结果 Y11/W10 一直没有时钟 -> mic_locked 不置位, led0/2/3 全灭。
//   * TDOA 低通 (TDOA_LPF_SH): 抑制每路亚样本 TDOA 的测量噪声 (见第 5b 节)
//   * 灯环直驱 (2026-10-11 简化): 只要 ref_energy > ANY_SND_E("有声音") 且有
//     解算结果, 就点亮该扇区的 1+2*RING_SPAN 颗灯; 超过 RING_BLANK_MS 没有
//     新解算 -> 全灭。 没有投票滤波器、没有走马灯、质量门限也不参与显示;
//     SECT_OFF 用于校准安装旋角, 质量只体现在 leds[2]/leds[3] 和诊断包里
//   * 定位精度系统优化 (2026-10-11, 原理见 doa_solver.v 的 ACCURACY BUDGET):
//       - 前端带通 + 预加重 (FILT_EN, mic_capture.v 的 mic_cond):
//         去直流/隆隆声 + 频谱白化, 相关峰更尖, 抗混响与抗低频漂移
//       - 每通道时延校正 TRIM0..TRIM5: 消除通道间固定时延偏差
//       - 抗差最小二乘: 残差超过 DOA_R_MAX 的通道剔除后重新拟合,
//         抑制混响/局部噪声造成的离群
//       - 质量门限 CONF_MIN: confidence 变为 0..255 的"麦克风一致性"质量分,
//         不一致时灯环不点亮, 投票滤波器也不采纳该次解算
//       - UART_PKT_MODE = 1: 输出 DOA 诊断包 (每路 tau / 方位 / 质量 /
//         扇区 / 残差 / 有效掩码), 用于标定与区分"麦克风问题/算法问题"
//
//   PMOD LED indicators (LVCMOS33, active high).  The default build
//   (LED_SELFTEST = 0) drives the live indicators.  READ leds[2] FIRST: it
//   selects which meaning leds[1] and leds[3] carry.
//   (2026-10-14: led1/led2 were reworked after "led2/led3 dark with led0/led1
//    healthy" proved ambiguous.  2026-10-15: the SAME report came back, so
//    the words on the lamps were split further -- led1's blink RATE and
//    led3's no-peak meaning now separate "wrong MAP6" from "dead array",
//    which no combination of the old lamps could.  See section 7.)
//     leds[0] = mic_locked (framing acquired -- this only proves the FPGA
//               generates BCK/WS, NOT that the mics answer)
//   --- leds[2] LIT: a peak exists AND sound is present -> "how good" ---
//     leds[1] = the CENTRE REFERENCE (ch6) is delivering data (solid)
//     leds[3] = that solution reached quality >= CONF_MIN
//   --- leds[2] DARK: no peak -> "what is missing" ---
//     leds[1] = reference status by BLINK RATE (slower = healthier):
//                 solid   the centre mic (the MAP6 phase) is moving
//                 ~1.5 Hz it is not, but a RING mic is
//                 ~3 Hz   nothing anywhere is moving
//     leds[3] = the OTHER lane-3 phase is delivering data
//               => leds[0] lit + leds[1] blinking + leds[3] LIT means the
//                  centre microphone is strapped to the phase MAP6 did not
//                  select: put MAP6 on the other of 6 / 7.  That is how
//                  MAP6 = 7 was chosen on 2026-10-15.
//   With LED_SELFTEST = 1 the fixed wiring self-test pattern is used:
//     leds[0] = 1 constant on       leds[1] = 1 Hz blink (0.5 s / 0.5 s)
//     leds[2] = 0 constant off      leds[3] = 1 constant on
//
//   I2S MASTER/SLAVE
//   ----------------
//   The FPGA is the I2S MASTER.  The array board has no clock source of its
//   own: every MSM261S4030H0 takes SCK from J32 pin 8 (MIC_CK) and WS from
//   pin 7 (MIC_WS), so BCK = clk/16 = 3.125 MHz and WS = BCK/64 = 48.83 kHz
//   are generated here and driven out of those two pins.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module audio_top #(
    parameter integer DATA_W    = 24,
    parameter integer DECIM     = 1,
    parameter integer FRAC_W    = 10,      // must match tdoa_engine / doa_solver
    parameter integer ACC_W     = 40,      // tdoa_engine accumulator width
    // MAC operand right-shift inside tdoa_engine.
    //
    // The engine stores 16-bit samples and the MAC reads them as (x >>> SHIFT),
    // so SHIFT sets how many significant bits the CORRELATION actually sees.
    // It was 4, i.e. a total of 8+4 = 12 bits off the 24-bit sample, and at the
    // speech levels this array actually produces (ref_energy around 128, i.e.
    // W = 7) that leaves rs_s with an RMS of only 0.7 LSB -- the correlator was
    // effectively running on 1 bit, which pointlessly inflates the sub-sample
    // jitter of every lag estimate.
    //
    // Measured with a bit-accurate Python model of this chain (see
    // FIXES_2026-10-16.md section 9): the bearing jitter from quantisation
    // alone falls from 2.2 deg to 1.4 deg at weak levels, and the strong-signal
    // case is unaffected (it was never quantisation-limited).
    //
    // Cost: 2*(STORE_W - SHIFT) grows by 4, so ACC_W must grow with it:
    //     ACC_W >= 2*(16-2) + log2(256) + 1 = 37   ->  40 leaves margin.
    // The engine's ref_energy output is scaled back by >>4 so that ANY_SND_E
    // and the W= field keep exactly the same meaning as before.
    parameter integer MAC_SHIFT = 2,
    parameter integer N         = 256,     // correlation window
    // Correlation lag range, +-LAG_RANGE samples.  This MUST cover the
    // largest physical TDOA, tau_max = R/c = 40mm / 343m/s = 116.6 us:
    //     at 16 kHz that is 1.87 samples  ->  L=2 would do
    //     at 48 kHz that is 5.60 samples  ->  L must be 6
    // mic_capture runs with DECIM=1 and the frame rate is BCK/64, so with
    // the array's ~3.1 MHz BCK the real rate is ~48 kHz, and L=2 clamped
    // every peak to the search boundary.  L=6 costs (2*6+1)/(2*2+1) = 2.6x
    // the correlation work and still refreshes at several kHz.
    parameter integer LAG_RANGE = 6,
    parameter integer NSEC      = 12,      // LED ring sectors
    // ---- filtering / ring calibration (2026-10-11) ----
    parameter integer TDOA_LPF_SH = 2,     // TDOA low-pass: avg += (x-avg)/2^SH
                                           // 0 = bypass, 1 = light,
                                           // 2 = default (~4-pass window),
                                           // 3 = heavier smoothing
    // Per-pass slew limit on every channel's TDOA, in Q(FRAC_W) samples
    // (1024 = 1.0 sample).  0 = unlimited.
    //
    // This is what stops "nudge the talker and the ring jumps to the far side
    // and stays there".  A 180-degree bearing flip means the correlator picked
    // every channel's peak on the WRONG side of zero, so all six taus jump by
    // about twice their true value on the SAME pass.  The plain IIR average in
    // tdoa_filter has no defence against that: one pass moves it almost all
    // the way over, and it then holds the wrong bearing for several passes.
    // Capping the movement per pass means a wrong peak must be seen repeatedly
    // before it can take over.
    //
    // 1 sample/pass is 10.2 deg per 84 ms pass = ~120 deg/s, faster than
    // anyone walks across a room, so normal following is not slowed down.
    // Lower it (e.g. 512) for a stiffer display; raise it if tracking feels
    // sluggish; 0 restores the original behaviour.
    parameter integer TDOA_SLEW   = 1024,
    parameter integer SECT_OFF_DEG = 67,   // LED-ring mounting rotation, in
                                           // DEGREES (signed).  Positive
                                           // rotates the displayed sector the
                                           // same way a positive bearing runs.
                                           //
                                           // 2026-11-01: 45 -> 67.  The
                                           // derivation below still says 75 for
                                           // a source exactly on a microphone
                                           // axis; 67 is that derivation minus
                                           // the 8 degrees by which the measured
                                           // source sat off the axis, so BOTH
                                           // the theoretical phi_k and the
                                           // measured direction land inside the
                                           // same sector with ~7 degrees of
                                           // margin on each side.  See "SET
                                           // FROM THE PHYSICAL LAYOUT".
                                           //
                                           // 2026-10-16: this replaced an
                                           // integer SECT_OFF that could only
                                           // shift by WHOLE 30 deg sectors --
                                           // too coarse for a hand-placed
                                           // ring (typical error 10..25 deg),
                                           // and its comment disagreed with
                                           // the code about the sign, so it
                                           // was a trap as well.
                                           //
                                           // The rotation is now applied
                                           // inside doa_solver, before the
                                           // sector quantiser, and can null
                                           // any offset to better than 1 deg.
                                           // It does NOT move `bearing` or the
                                           // diagnostic packet: those stay in
                                           // the MICROPHONE frame, which is
                                           // what a calibration compares
                                           // against a known source position.
    //------------------------------------------------------------------
    //  The other two calibration knobs.
    //
    //  Both already existed as module parameters (doa_solver.MIRROR_X and
    //  sk9822_ring.LED_REVERSE) but were NOT connected here, so top.v always
    //  used their defaults and a calibration could not actually turn them on
    //  -- `CALIBRATION.md` and tools/calib.py both told you to write
    //  `.MIRROR_X(1)` into this instantiation, which would not have compiled.
    //  Exposed 2026-10-16 so the documented procedure is literally executable.
    //
    //  MIRROR_X   : the array's chirality -- a REFLECTION about the 90/270 deg
    //               axis.  Set it when walking the source from one known angle
    //               to another makes the reported bearing move the WRONG WAY
    //               (calib.py decides this from two points).  A rotation
    //               cannot fix a reflection and vice versa, so this comes
    //               BEFORE SECT_OFF_DEG.
    //  LED_REVERSE: swaps the order of the 12 LEDs along the strip.  Only
    //               needed if the lit group appears mirrored about sector 0.
    //
    //  ---- SET FROM THE PHYSICAL LAYOUT (2026-10-18, CORRECTED 2026-11-01)
    //
    //  Required mapping (board owner, MEASURED on 2026-11-01 -- the 10-18
    //  table had every group one LED too far round, i.e. centred on 2k+1
    //  instead of 2k, which is why a source in front of mic3 lit nothing that
    //  looked right even after the polarity was fixed):
    //      mic0 -> LEDs 11,0,1     mic3 -> LEDs 5,6,7
    //      mic1 -> LEDs  1,2,3     mic4 -> LEDs 7,8,9
    //      mic2 -> LEDs  3,4,5     mic5 -> LEDs 9,10,11
    //  i.e. mic k -> the three LEDs centred on 2k, on a ring whose LED
    //  numbers run CLOCKWISE.  (mic0 wraps through 11 because 2*0-1 = -1.)
    //
    //  Why that needs BOTH knobs and cannot be done with an offset alone:
    //  doa_solver's sector index increases COUNTER-clockwise
    //      sector = floor((theta + SECT_OFF_DEG + 180) / 30)
    //  while the strip numbers increase CLOCKWISE.  Turning one into the
    //  other is a REFLECTION, not a rotation -- that is what LED_REVERSE is
    //  for; SECT_OFF_DEG can then only shift the phase.
    //
    //  With LED_REVERSE = 1 the lit PHYSICAL indices for sector s are
    //  {10-s, 11-s, 12-s}, centred on 11-s.  Requiring 11-s = 2k gives
    //  s = 11-2k, and putting theta = phi_k = 90-60k through the quantiser
    //      (90 - 60k + SECT_OFF_DEG + 180) / 30  =  (11 - 2k) + 0.5
    //  solves to SECT_OFF_DEG = 75.  The HALF sector matters: with 60 the six
    //  microphone directions land exactly on bin edges, so a fraction of a
    //  degree flips the group; a half-sector offset centres them.
    //
    //  67 instead of 75 is the same derivation minus the measured 8 degree
    //  offset of the actual source from the mic3 axis, so that the six axes
    //  AND the direction that was measured on the bench both fall inside the
    //  intended sector.  Re-derive it if the array is ever remounted.
    //
    //  Checked in simulation (tb_sk9822 "ring map" cases, tb_doa_solver
    //  test 7): all six microphones light exactly the table above.
    //  If the ring ever appears mirrored on hardware, flip LED_REVERSE to 0
    //  and re-derive SECT_OFF_DEG -- do NOT just nudge the offset.
    //------------------------------------------------------------------
    parameter integer MIRROR_X    = 0,
    parameter integer LED_REVERSE = 1,
    //  BEARING_FLIP: polarity of the WHOLE fit, 180 degrees.  See
    //  doa_solver.FLIP_180 for what it is and why it cannot change the
    //  quality field.  Set on 2026-11-01 because a source standing in front
    //  of mic3 (theta = -90) was reported as theta = +90 -- the ring lit the
    //  side OPPOSITE the talker.
    //
    //  This is a ROTATION, so unlike MIRROR_X it is compatible with any
    //  SECT_OFF_DEG; the two are independent knobs and both were needed here.
    //  Set to 0 to restore the 2026-10-16 behaviour and watch the ring go
    //  back to the far side, which is the quickest way to confirm the
    //  diagnosis on hardware.
    //  ★ 2026-11-01 现场实测: 1 -> 0.
    //    设置 1 的依据是"站在 mic3 前, 灯亮到对侧" → 判定整帧极性反了 180 度。
    //    但那是 PEAK_ABS 之前的数据: 当时只有正相关峰被采纳 (tdoa_engine
    //    里 `valid <= best > 0`), 反相通道的 lag 取到的是"最大正旁瓣",
    //    不是真值, 拟合出来自然是乱的。改成 |C| 之后六路 lag 全部有效,
    //    极性从"部分错"变成"全部正确", 那个 180 度补正就成了多余的。
    //
    //    实测证据 (用户两次独立标定, 都是精确 180 度):
    //      站在 mic3  → 串口 M=0, LED=11,00,01   (3 + 3 = 6 ≡ 0)
    //      站在 mic2  → 串口 M=5, LED=09,10,11   (2 + 3 = 5)
    //    mic 序号整体 +3 就是整张图转 180 度; 反射 (镜面) 得不到这个结果
    //    (镜面会让 mic3→mic3, mic2→mic1), 所以确认是"旋转"而不是"镜像",
    //    也就是这个旋钮, 不是 MIRROR_X, 不是 SECT_OFF_DEG。
    parameter integer BEARING_FLIP = 0,
    // ---- front-end conditioning (2026-10-11) ----
    parameter integer FILT_EN     = 1,     // 1 = 带通 + 预加重 (推荐, 见 mic_capture.v)
                                           // 0 = 直通, 用于 A/B 对比
    // ---- channel map, exposed here on purpose (2026-10-14) ----
    // mic_capture default: mic0..5 = ring (lane0/1/2 x WS-low/high),
    // mic6 = lane3 WS-LOW = the centre reference.  The one thing the
    // schematic does not pin down is which of the two chips sharing a lane
    // sits on WS-low, so if a brought-up board shows a silent or swapped
    // channel, THIS is the knob -- no other file needs touching:
    //   * centre microphone silent while the ring is alive -> try MAP6 = 7
    //     (lane3's other WS phase)
    //   * two adjacent ring mics swapped -> swap that lane's MAP pair
    // Verify with the UART status line: E= prints the raw level of all
    // seven channels, the LAST field being the reference, and D= says which
    // channels are moving at all.
    parameter [2:0] MAP0 = 3'd0,           // MIC_DAT0 WS-low  = mic0 (U7)
    parameter [2:0] MAP1 = 3'd1,           // MIC_DAT0 WS-high = mic1 (U8)
    parameter [2:0] MAP2 = 3'd2,           // MIC_DAT1 WS-low  = mic2 (U13)
    parameter [2:0] MAP3 = 3'd3,           // MIC_DAT1 WS-high = mic3 (U14)
    parameter [2:0] MAP4 = 3'd4,           // MIC_DAT2 WS-low  = mic4 (U19)
    parameter [2:0] MAP5 = 3'd5,           // MIC_DAT2 WS-high = mic5 (U20)
    // ---- MAP6: CHANGED 6 -> 7 on 2026-10-15, ON BOARD EVIDENCE ----
    //  The centre microphone's L/R strap on this board puts it in lane3's
    //  WS-HIGH phase, not WS-low.  Evidence: with MAP6 = 6 the first
    //  bring-up reported leds[0] = 1 (framing fine), leds[1] = BLINKING,
    //  leds[2] = 0, leds[3] = 0.  leds[1] is driven by
    //        ref_alive = ch_act[6]
    //  and ch_act[6] = 0 means slot6 has produced no moving sample, so the
    //  slot we were reading as the reference is dead -- a tristated lane3
    //  phase pulled to a constant zero.  That is exactly the "MAP6 = 7"
    //  case the old comment here predicted.
    //
    //  This is NOT a guess with downside: slot6 is already known dead, so
    //  selecting it costs nothing to give up, while slot7 is the only
    //  remaining place the centre microphone can be.  If the board is ever
    //  re-strapped, read the lamps (table at the LED OUTPUT STAGE) and put
    //  this back to 3'd6.
    parameter [2:0] MAP6 = 3'd7,           // lane3 WS-HIGH  (U1) = CENTRE
    // ---- robust fit / quality gate (2026-10-11) ----
    //  ★ 2026-11-01 调试值: 128 -> 96 -> 64.
    //    128 要求 4 颗内点且残差很小; 现场实测正常说话只到 Q=099, 也就是
    //    "刚好擦过 96", 所以灯一闪就灭。64 让 3 颗内点 + 中等残差就能点亮,
    //    与"正常音量稳定触发"这个要求对应。
    //
    //    为什么不再往下: Q<64 基本等于"内点 <= 2 颗", 两三个方程的拟合在
    //    圆周上到处都能自洽, 那时灯确实会亮, 但亮的是噪声的方向, 比不亮更
    //    误导。64 是这条线的下限, 不要为了"更容易亮"调到 32。
    //    历史值: 128 = 4 颗内点且残差<=1 sample; 160 = 更严; 3 颗内点理论上限
    //    只有 127, 所以 >=128 就等于"内点必须 >= 4 颗"。96 = 3~4 颗。
    parameter integer CONF_MIN    = 64,    // 解算质量门限 (同时门控灯环与 leds[3])
                                           // 质量 = 255*(1-残差/2^DOA_R_SH)*(内点数/6)
                                           // 内点 = 残差 <= DOA_R_MAX 的麦
    parameter integer DOA_R_MAX   = 1200,  // 残差离群阈值, Q(FRAC_W)=1/1024 sample
                                           // 1200 = 1.17 sample ≈ 12°
                                           // 2026-11-01 由 350 改来: 350 只有 0.34 sample,
                                           // 而单路 tau 的实测抖动 σ≈0.4 sample
                                           // (tools/sector_jitter.py), 于是 ~90% 的帧
                                           // 至少被剔掉一颗麦, 29% 的帧剔到只剩 2~3 颗,
                                           // 几何归一化在小样本上失稳 -> 方位跳到对面。
                                           // 改 1200 后 离群帧 0%, 帧级正确率 52%->93%。
    parameter integer DOA_R_SH    = 12,    // 质量降到 0 时的残差 = 2^DOA_R_SH = 4096
                                           // 必须 >= DOA_R_MAX, 否则 res_max 超过它
                                           // 会被 clamp 成 0, 质量门反而把好帧全毙掉
    // 每通道时延校正, Q(FRAC_W)=1/1024 sample。
    // 标定方法: UART_PKT_MODE=1 读回 tau_m 的均值, 与理想值
    //      tau_m = -5.694 * cos(theta - phi_m)     (R=40mm, c=343m/s, fs=48.828kHz)
    // 比较后直接填 TRIMm = (实测均值 - 理想值) * 1024。 硬件做的是
    // tau_corrected = tau_measured - TRIM, 所以"测得偏大"就填正数。
    // tools/doa_diag.py --ref-theta <已知方位> 会把这六个值算好打印出来。
    parameter integer TRIM0 = 0,
    parameter integer TRIM1 = 0,
    parameter integer TRIM2 = 0,
    parameter integer TRIM3 = 0,
    parameter integer TRIM4 = 0,
    parameter integer TRIM5 = 0,
    // ---- microphone sanity guard (2026-10-11) ----
    // A channel whose correlation peak sits more than MAG_MIN_SH halvings
    // (6 dB each) below the strongest microphone is dropped BEFORE the fit,
    // even if its peak was positive.  A weak, acoustically blocked,
    // mis-mapped or unclocked channel otherwise poisons the FIRST
    // least-squares pass -- and the robust pass then measures every
    // residual against that poisoned fit, so it can end up discarding the
    // GOOD microphones instead.  The guard has to act first.
    // 4 = drop anything 24 dB down; 16 or more disables the check.
    //  ★ 2026-11-01 调试值: 4 -> 5.  这一层砍掉"相关峰比最强路低 2^SH 倍"
    //    的通道, 4 = 24 dB。现场 V=09 说明六只环形麦只有两路出峰, 其余四路
    //    被这道门剔除, 只剩两个方程 -- 拟合必然不稳。5 (30 dB) 让偏弱的麦
    //    也能参与, 内点数上去了, 质量分才可能超过 CONF_MIN。
    //    16 = 关闭该检查; 再往上放会连噪声峰一起收进来。
    parameter integer MAG_MIN_SH = 5,
    // ---- LED ring display (2026-10-11, 只有两态) ----
    //   有声音 -> 对应扇区点亮 1+2*RING_SPAN 颗灯
    //   没声音 -> 全灭 (超过 RING_BLANK_MS 没有新方位)
    parameter integer RING_SPAN     = 1,     // 每个方位点亮 1+2*SPAN 颗 (1=3 颗)
    parameter integer RING_BLANK_MS = 800,   // 多久没有新方位就熄灭。说话停顿一般
                                             // 0.2~0.5s, 别再设小。
                                             // 2026-10-16: 500 -> 800。因为加了
                                             // DIR_MIN_MS=500 的更新节流, 两者相等
                                             // 会让"每 500ms 的刷新"恰好落在熄灭计
                                             // 时器到期的同一刻 -> 灯环在临界点闪断。
                                             // 必须明显大于 DIR_MIN_MS。
    parameter integer DIR_MIN_MS    = 500,   // 灯环更新节流: 触发一次后 HOLD 这么久,
                                             // 期间忽略新的解算结果。
                                             // 解算周期约 84ms, 不节流时灯环会跟着
                                             // 抖动的方位每 84ms 跳一次。
                                             // 0 = 关闭节流(每次解算都更新)。
                                             // 关系: 收到声音 -> 立即刷新并锁
                                             // DIR_MIN_MS -> 期间不接收 -> 到点后
                                             // 若仍有声音才再次刷新。
    //  ---- 方位确认 (2026-10-18 新增, 治"说话时灯到处乱跳") ----
    //
    //  节流只限制了更新**频率**, 没有限制更新**内容**: 每到 0.5s 窗口结束,
    //  它就把"下一帧通过质量门的解算"直接推给灯环 —— 而那一帧的方位可能
    //  和上一帧差 180°(相关器选到反相峰 / tau 还没收敛 / 抗差拟合剔掉了
    //  不同的麦)。板上表现就是"说一句话, 灯环在环上乱跳"。
    //
    //  确认机制: 新扇区必须被**连续** SECT_VOTES 帧解算一致地选中, 才允许
    //  显示; 若新扇区与当前显示位置相差 >= SECT_JUMP_SECT 个扇区(默认 3 =
    //  90°), 视为"大跳", 要求 SECT_VOTES_FAR 帧 —— 因为真实声源在 0.5s 内
    //  转过 90° 是可能的, 但远不如相关器出错常见。
    //
    //  好处: 孤立的一两帧错误既不会改灯, 也不会被 IIR 记住后反复跳;
    //  代价: 真实转向要晚 3~5 个解算周期(约 0.25~0.42s)才显示。
    //  把 SECT_VOTES 设成 1 就退回旧行为(每帧都认)。
    parameter [3:0]   SECT_VOTES     = 4'd3, // 同扇区连续几帧才换灯
    parameter [3:0]   SECT_VOTES_FAR = 4'd5, // 大角度跳变要几帧
    parameter [3:0]   SECT_JUMP_SECT = 4'd3, // 超过几个扇区算"大跳"(3 = 90°)

    //  (REPORT_MS removed 2026-11-01.)  The periodic report used to have its
    //  own 5 s timer, which meant the terminal and the lamp ring disagreed
    //  about both the rate and the content.  It is now paced by the ring's own
    //  refresh strobe (dir_update, throttled by DIR_MIN_MS = 500 ms) and
    //  prints the human line
    //      ANG=180deg  LED=05,06,07  M=3  Q=200
    //  so "what the log says" and "what the lamps show" are the same frame.
    // ---- FIRST-ARRIVAL EVENT REPORT (2026-11-01) ---------------------
    //  Every time sound starts, first_arrival latches the microphone that
    //  received the wave FIRST and emits one UART line:
    //
    //      A M=<mic> L=<led>,<led>,<led> T=<tau> S=<sector> G=<ring lit>
    //
    //  Only the first plane wave of an utterance is reported: the event is
    //  armed again only after the room has been quiet for EVT_GAP_MS, so a
    //  single sentence produces exactly one line rather than one per 84 ms
    //  solved frame.
    //
    //  EVT_GAP_MS is that re-arm delay.  It must be longer than the pauses
    //  inside normal speech (~200..400 ms) or one sentence is reported several
    //  times, and shorter than the gap between two sentences or the second one
    //  is never reported.  600 ms sits between the two.  Set it to 1 to report
    //  on almost every solved frame (useful for bringing the link up), or to
    //  0 to disable the event report entirely.
    parameter integer EVT_GAP_MS    = 600,

    //  ★ 2026-11-01 调试值: 32 -> 8 -> 4 (= W=2).
    //    32 要 W>=6 才开闸, 而现场 W 长期落在 2..8, 门限正压在环境噪声上;
    //    8 (W>=3) 之后解算器能启动了, 但用户反馈"要很大声才点亮", 因为
    //    W 是 log2 域, 每 1 个码 = 6 dB, 从 3 抬到 4 就是 6 dB 的响度差。
    //    4 (W>=2) 让正常说话就能开闸。
    //
    //    这一层只决定"要不要启动解算", 不决定"点亮不点亮" -- 后者是
    //    CONF_MIN。所以放宽它是安全的: 安静时解算器多跑几次, 质量门照旧
    //    把噪声挡在灯环外面。
    parameter integer ANY_SND_E     = 4,     // "有声音"的能量门: 参考麦相关窗的
                                             // ref_energy 超过此值才允许驱动灯环、
                                             // 启动解算。ref_energy = RMS(x16)²,
                                             // x16 = 24bit/256。
                                             //
                                             // 2026-10-16 修正 -- 这个常量原来是
                                             // 1024, 注释按"原始值 8192 LSB ≈ 62
                                             // dB SPL"换算。但 ref_energy 走的是
                                             // **mic_cond 滤波后**的信号, 而该链
                                             // (DC-block + 预加重 0.5-0.46875z⁻¹ +
                                             // 低通) 是转折约 500 Hz 的高通:
                                             //     白噪 -10 dB / 1 kHz -23 dB /
                                             //     400 Hz -28 dB / 100 Hz -34 dB
                                             // 语音能量正好集中在这一段, 所以 1024
                                             // 把有效门限抬到了 raw 侧 75~88 dB SPL。
                                             // 板上症状就是"E= 看着很正常(14~17) 而
                                             // led2/led3 永远不亮"。
                                             //
                                             // 1024 -> 128 修掉了那个量纲错, 但现场
                                             // 反馈仍是"要很大声才亮": 实测静音只有
                                             // W=0(偶见 W=3), 说话才 W>=7, 而 128
                                             // 正好是 W=7 -- 正常说话只是勉强压线。
                                             // 32 (= W=5) 让说话轻松过门, 同时仍比
                                             // 实测静音上限(W=3, ref_energy<16)高
                                             // 一倍, 不会把安静时的噪声放进来。
                                             //
                                             // 调参规则(不需要猜): 看状态行的 W= 字段,
                                             // 静音记一个数 n, 就取 ANY_SND_E = 2^(n+2),
                                             // 即比静音底噪高 12 dB。
                                             //   还想更灵敏 -> 16 (W=4); 静音乱亮 -> 64 (W=6)
                                             // 0 = 不判断(噪声也会点亮)。
    // ---- UART payload ----
    // 诊断模式的触发是 diag_stb(= doa_done), 波形模式的触发是 ch_stb:
    //   模式 1/2 收到任何一行/一包 -> doa_solver 确实在跑
    //   模式 1/2 完全没输出        -> solver 从未启动, 故障在上游(麦克风数据)
    // 2026-10-14: 模式 2 现在还会**每 ~84ms 输出一条状态行, 不等解算**,
    // 所以"完全没有解算"时串口也有话说:
    //   N=1A2B D=7F E=090A0A090B0B09
    //   N = 音频帧计数 (不动 = 根本没有帧)
    //   D = 哪几路原始数据还在动 (bit k = ch k); D=00 = 一路数据都没有
    //   E = 七路原始电平码, 每路 2 个 hex, **最后一个是中心参考麦 ch6**,
    //       码值 = log2(平均|x|), 每格 6 dB: 00 = 恒为 0(死路/没接),
    //       ~09 = 正常 MEMS 底噪, 17+ = 大声。 (低于 256 LSB 也显示 00)
    // 典型判读:
    //   E 前 6 组有值 + 最后一组 00 -> 参考麦这一路坏(或 MAP6 填错/WS 相位反),
    //     此时所有相关值恒为 0, 解算器不启动 -> led2/led3 全灭
    //   D=00 或 N 不动            -> 数据根本没进来: 查 MIC_D0..D3 / 供电 / CK
    //   E 全部有值但很小(<=07)     -> 电平太低: 先 FILT_EN=0 做 A/B
    parameter integer UART_PKT_MODE = 2,   // 0 = 原始 7 通道波形包 (A5 5A)
                                           // 1 = DOA 诊断二进制包 (A5 5B),
                                           //     配套 tools/doa_diag.py
                                           // 2 = DOA 文本诊断(ASCII), 任意串口终端
                                           //     (115200 8N1) 直接可读, 约 12 行/秒:
                                           //   Q=CB R=14 V=3F S=07 L=0A55C
                                           //   T=0000,FB12,FB12,0000,04EE,04EE
                                           // Q=质量(hex,越大越好) R=最大残差(Q8)
                                           // V=有效掩码(哪路没出真峰) S=扇区
                                           // L=六路电平码(18bit, 3bit/路)
                                           // T=六路 tau (Q8 有符号, 1 LSB=1/256 采样;
                                           //   上面这组 T 就是声源在 theta=0 的
                                           //   理想值, 可当自检基准:
                                           //   tau_m = -5.694*cos(theta-90+60m))
    // ---- LED mode (2026-10-11, ladder updated 2026-10-14) ----
    parameter integer LED_SELFTEST = 0     // 0 (默认) = 实时指示, 见第 7 节.
                                           //   四级阶梯, 从左往右第一个不对的就是故障点:
                                           //     led0 锁定 (BCK/WS 有输出)
                                           //     led1 参考麦 ch6 有数据(常亮) / 没数据(闪)
                                           //     led2 引擎找到了正相关峰(any_valid)
                                           //     led3 有峰时=质量达标; 无峰时=参考麦电平
                                           //          够(均值>=256 LSB, 高于引擎 >>8 截断线)
                                           // 1 = 固定自检: led0/led3 常亮,
                                           //     led1 1Hz 闪, led2 常灭 (查引脚/接线)
) (
    input  wire        clk,             // 50 MHz  (V22)
    input  wire        rst_n,           // reset key (AA13, active low)

    // ---- mic array (I2S master: FPGA drives CK/WS, mics return data) ----
    output wire        mic_array_bck,   // A13 (flying lead, MIC_CK) - driven
    output wire        mic_array_ws,    // A14 (flying lead, MIC_WS) - driven
    input  wire [3:0]  mic_array_d,     // C13,B16,B15,B13 = MIC_D3..MIC_D0

    // ---- LED ring on the mic array board (SK9822 x12, single 2-wire bus) ----
    output wire        mic_led_clk,     // C14 (flying lead, LED_CK)
    output wire        mic_led_dat,     // C15 (flying lead, LED_DA)

    // ---- UART to PC ----
    output wire        uart_txp,        // U15

    // ---- PMOD LEDs ----
    output wire [3:0]  leds,

    // ---- [新增 2026-11-02] DOA 结果外引给视频侧 pan_ctrl ----
    //   原先 bearing/sector/confidence/doa_done 全是模块内部信号，只送到 UART 文本
    //   和灯环，视频侧拿不到方位角 —— 这是"自动转向"的硬阻塞，必须先补这四个端口。
    //   位宽/语义与 doa_solver 输出一致（不改内部任何逻辑）。
    output wire signed [17:0] doa_bearing_o,   // Q2.16：32768 == 90°
    output wire [3:0]         doa_sector_o,    // 0..11，每扇区 30°
    output wire [7:0]         doa_conf_o,      // 0..255，越大越可信
    output wire               doa_valid_o      // 单拍：本块解算完成
);

    // Hold-counter width, shared by DIR_MIN_MS (the ring throttle), ANYV_HOLD
    // and LED_HOLD.  25 bits holds 33_554_431, i.e. 0.67 s at 50 MHz -- enough
    // for a 0.5 s window (DIR_MIN_MS = 500 needs 25_000_000).
    //
    // It lives here, immediately inside the module body, because a localparam
    // may NOT appear inside the  #( ... )  parameter list (that is a syntax
    // error, not a style question).
    localparam integer LHW = 25;

    //==================================================================
    // 1.  Clock sources
    //
    //     PR1014 IS NOT A DEFECT ON THIS DEVICE -- read this before trying to
    //     "fix" it.  THIS project's fitter really does emit it
    //     (impl/pnr/audio.log, line 16):
    //
    //         WARN (PR1014) : Generic routing resource will be used to clock
    //                         signal 'clk_d' by the specified constraint.
    //
    //     The obvious remedy is to instantiate a global clock buffer, but on
    //     GW5A THERE IS NO USER-INSTANTIABLE ONE:
    //       * the GW5A primitive library (IDE/simlib/gw5a/prim_sim.v, 134
    //         modules) contains no BUFG and no TCLKBUF -- only CLKDIV,
    //         CLKDIV2, DCE, DCS, DDRDLL and DLLDLY;
    //       * instantiating the generic `BUFG` is rejected outright:
    //             ERROR (EX3937) : Instantiating unknown module 'BUFG'
    //       * Gowin's OWN bundled example IDE/bin/Documents/assign_led_on
    //         emits the very same PR1014 for its single clock.
    //
    //     So global-versus-generic clock routing is the fitter's decision on
    //     this family and the warning is informational.  Leave it alone.
    //
    //     --- CORRECTION, 2026-10-11 -------------------------------------
    //     This block used to end with:
    //         "The measured result backs that up: Fmax 68.4 MHz against a
    //          50 MHz constraint with no negative slack.  Leave it alone."
    //     and attributed the warning to impl/pnr/micarray.log.
    //
    //     BOTH HALVES OF THAT WERE WRONG AND HAVE BEEN REMOVED.
    //       * There is no micarray.log under impl/pnr/ -- the file is
    //         audio.log.  The sentence was carried over from a different
    //         project's report.
    //       * "Fmax 68.4 MHz ... no negative slack" contradicted this
    //         project's own report.  The truth, from
    //         impl/pnr/audio.timing_paths, was:
    //             Setup WNS = -9.055 ns   (fails; Fmax ~34 MHz)
    //             Hold  WNS = -1.012 ns   (fails)
    //         A reviewer who trusted the comment would have signed off a
    //         bitstream that does not meet 50 MHz.
    //
    //     Root causes and fixes (both applied 2026-10-11):
    //       * setup: the flat combinational magnitude guard in section 5a --
    //         all six worst paths ran u_tdoa/valid_flat_* ->
    //         u_tdoa_lpf/out_valid_*.  Now three registered stages.
    //       * hold: mic_bck <-> clk were declared as unrelated PRIMARY
    //         clocks, so the fitter timed the toggle-synchroniser crossing
    //         as if the two clock trees were edge-aligned.  Now declared
    //         asynchronous in audio.sdc.
    //
    //     RULE: any timing claim written in this file must be readable in
    //     THIS project's impl/pnr/audio.timing_paths.  Never quote another
    //     build's numbers.
    //==================================================================
    wire clk_g;          // 50 MHz system clock
    wire bck_i;          // bit clock used by the capture chain (FPGA-generated)
    wire ws_i;

    assign clk_g = clk;      // no buffer primitive exists; see the note above

    //------------------------------------------------------------------
    //  THE FPGA IS THE I2S MASTER  (fixed 2026-10-11)
    //
    //  The first revision defaulted to "the array generates BCK/WS by
    //  itself".  The module schematic (doc/sch_mics.png) shows every
    //  MSM261S4030H0 taking SCK from J32 pin 8 (MIC_CK) and WS from
    //  pin 7 (MIC_WS), with no clock source anywhere on the board: they
    //  are plain I2S SLAVES.  With that assumption there was no BCK at
    //  all, so `mic_locked` never asserted (leds[0] stayed dark) and
    //  every downstream block stayed idle (leds[2]/leds[3] dark).
    //
    //  Timing: 50 MHz / 16 = 3.125 MHz BCK, 32 BCK per half frame
    //          -> WS = 3.125 MHz / 64 = 48.83 kHz sample rate.
    //          Aperture R = 40 mm, so tau_max = R/c = 116.6 us
    //          = 5.69 samples, which is inside LAG_RANGE = 6.
    //------------------------------------------------------------------
    wire bck_gen;
    clk_div #(.DIV(16)) u_bck_div (
        .clk_in (clk_g),
        .rst_n  (rst_n),
        .clk_out(bck_gen)
    );

    assign bck_i         = bck_gen;
    assign mic_array_bck = bck_gen;

    //==================================================================
    // 1b.  Reset synchronisers, one per clock domain
    //
    //      rst_n comes from a push button, so its release edge is
    //      asynchronous to BOTH clocks and can land inside a flip-flop's
    //      recovery/removal window.  Each domain therefore gets the classic
    //      "assert asynchronously, release synchronously" pair: the first
    //      flop may go metastable, the second resolves it, and each domain
    //      then leaves reset on its own clock edge instead of at an
    //      arbitrary instant.
    //==================================================================
    reg rstn_clk_meta, rstn_clk_sync;
    always @(posedge clk_g or negedge rst_n) begin
        if (!rst_n) begin
            rstn_clk_meta <= 1'b0;
            rstn_clk_sync <= 1'b0;
        end else begin
            rstn_clk_meta <= 1'b1;
            rstn_clk_sync <= rstn_clk_meta;
        end
    end
    wire rst_n_clk = rstn_clk_sync;

    reg rstn_bck_meta, rstn_bck_sync;
    always @(posedge bck_i or negedge rst_n) begin
        if (!rst_n) begin
            rstn_bck_meta <= 1'b0;
            rstn_bck_sync <= 1'b0;
        end else begin
            rstn_bck_meta <= 1'b1;
            rstn_bck_sync <= rstn_bck_meta;
        end
    end
    wire rst_n_bck = rstn_bck_sync;

    // One WS edge per 32 BCK.  DELIBERATELY CLOCKED ON THE **FALLING** EDGE
    // OF BCK -- do not "tidy" this back to posedge.
    //
    // MSM261S4030H0 (DS-035, p5 timing diagram) specifies, both referenced to
    // the SCK RISING edge:
    //     tWSS  WS setup = 0 ns
    //     tWSH  WS hold  = 20 ns MIN
    // so the microphones latch WS on the SCK rising edge and WS must not move
    // for 20 ns afterwards.
    //
    // This counter used to run on posedge bck_i, which placed the WS
    // transition one clk-to-out (~10 ns, as the old comment here admitted)
    // AFTER a BCK rising edge: ~10 ns of hold against a 20 ns requirement.
    // The failure mode is worse than a metastable channel select.  Lanes 0..2
    // each carry TWO microphones that decide "is this my half?" from WS, so a
    // microphone sampling WS ambiguously can drive SD while its neighbour is
    // still driving the same wire -- two push-pull outputs fighting.  tWSH
    // exists to prevent precisely that.
    //
    // Counting on the falling edge moves the transition into the middle of
    // the SCK low phase: ~160 ns of setup before the next rising edge and
    // ~160 ns of hold after the previous one, i.e. 8x the tWSH requirement.
    //
    // The receiver is NOT disturbed by this change.  i2s_rx_8slot samples ws
    // with posedge bck_i, and in BOTH arrangements the transition falls
    // between the same pair of rising edges, so ws_r captures the new level
    // on the same clock and the bit alignment is unchanged.  tb_i2s_rx_8slot
    // was extended in the same change to model this phase (its master now
    // emits WS on the falling edge) AND to assert the tWSS/tWSH margins --
    // the old testbench generated WS on the rising edge, i.e. it reproduced
    // the bug rather than catching it.
    //
    // Reset safety: rst_n_bck is released synchronously to posedge bck_i, so
    // it deasserts ~160 ns BEFORE the next negedge and this flop leaves reset
    // with a clean recovery/removal margin.
    reg [5:0] ws_div;
    always @(negedge bck_i or negedge rst_n_bck) begin
        if (!rst_n_bck) ws_div <= 6'd0;
        else            ws_div <= ws_div + 1'b1;
    end

    assign ws_i         = ws_div[5];
    assign mic_array_ws = ws_div[5];

    //==================================================================
    // 2.  Capture  (bck domain)
    //==================================================================
    wire signed [DATA_W-1:0] ch0, ch1, ch2, ch3, ch4, ch5, ch6;
    // Conditioned copies of the same channels, for the TDOA engine only.
    // The raw channels keep feeding the UART: the PC tool has to see what
    // the microphones actually output, not what the conditioning chain made
    // of it, otherwise a filter problem is indistinguishable from a
    // microphone problem.
    wire signed [DATA_W-1:0] fch0, fch1, fch2, fch3, fch4, fch5, fch6;
    // sub-band PHAT analysis outputs (2026-10-19): NBAND bands x 7 channels,
    // packed flat -- band b of channel c at [NBAND*DATA_W*c + DATA_W*b +: DATA_W]
    localparam integer NBAND = 4;
    wire signed [NBAND*DATA_W*7-1:0]   band_out;
    wire signed [NBAND*DATA_W-1:0]     ref_b  = band_out[NBAND*DATA_W*6 +: NBAND*DATA_W];
    wire signed [NBAND*DATA_W*6-1:0]   mic_b  = band_out[NBAND*DATA_W*0 +: NBAND*DATA_W*6];
    wire                     ch_stb;
    wire [15:0]              frame_cnt;
    wire                     mic_locked;
    // the lane-3 phase MAP6 did NOT select -- the cross-check for the centre
    // microphone's L/R strap (see mic_capture.slot_l3_other and the LED table)
    wire signed [DATA_W-1:0] slot_l3_other;

    mic_capture #(
        .DATA_W (DATA_W),
        .DECIM  (DECIM),
        .FILT_EN(FILT_EN),
        .MAP0   (MAP0), .MAP1 (MAP1), .MAP2 (MAP2), .MAP3 (MAP3),
        .MAP4   (MAP4), .MAP5 (MAP5), .MAP6 (MAP6)
    ) u_capture (
        .rst_n    (rst_n_bck),
        .bck      (bck_i),
        .ws       (ws_i),
        .d_in     (mic_array_d),
        .ch0(ch0), .ch1(ch1), .ch2(ch2), .ch3(ch3),
        .ch4(ch4), .ch5(ch5), .ch6(ch6),
        .fch0(fch0), .fch1(fch1), .fch2(fch2), .fch3(fch3),
        .fch4(fch4), .fch5(fch5), .fch6(fch6),
        .band_out (band_out),
        .ch_stb   (ch_stb),
        .frame_cnt(frame_cnt),
        .slot_l3_other(slot_l3_other),
        .locked   (mic_locked)
    );

    //==================================================================
    // 3.  Cross ch_stb (bck domain) into the clk domain.
    //
    //     ch0..ch6 are held stable by mic_capture for a whole frame --
    //     ~20 us at 48 kHz, against the 2-3 clk the toggle synchroniser
    //     needs -- so no FIFO is required.  That is a REAL contract, not an
    //     accident, so micarray.sdc bounds the data paths with
    //     set_max_delay -datapath_only instead of relying on
    //     set_clock_groups to hide them from STA entirely.
    //
    //     A one-shot `stb_pipe` shift register used to be declared here and
    //     never used; any reader would have believed a 3-stage synchroniser
    //     existed.  It is gone.
    //==================================================================
    reg stb_tgl_bck, stb_tgl_meta, stb_tgl_sync, stb_tgl_d;

    always @(posedge bck_i or negedge rst_n_bck) begin
        if (!rst_n_bck) stb_tgl_bck <= 1'b0;
        else if (ch_stb) stb_tgl_bck <= ~stb_tgl_bck;
    end

    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            stb_tgl_meta <= 1'b0;
            stb_tgl_sync <= 1'b0;
            stb_tgl_d    <= 1'b0;
        end else begin
            stb_tgl_meta <= stb_tgl_bck;
            stb_tgl_sync <= stb_tgl_meta;
            stb_tgl_d    <= stb_tgl_sync;
        end
    end

    wire ch_stb_clk = stb_tgl_sync ^ stb_tgl_d;

    //==================================================================
    // 3b. RAW CHANNEL HEALTH MONITOR   (clk domain, 2026-10-14)
    //
    //     WHY THIS EXISTS
    //     ---------------
    //     "led2 dark" has two very different causes and the board could not
    //     tell them apart:
    //       a) the microphones are not delivering data at all; or
    //       b) they deliver data, but the correlations never see it
    //          (dead REFERENCE channel, wrong MAP, filter killing the level).
    //     Every pre-existing diagnostic looks at the CONDITIONED path
    //     (fch*, ref_energy) and at the six RING microphones only -- the
    //     CENTRE microphone, which is the TDOA reference, was invisible.
    //     A silent reference makes every correlation exactly 0, so the
    //     engine reports best = 0 (its validity rule is best > 0),
    //     any_valid = 0, the solver never even starts, and the board shows
    //     led0 (framing) and led1 (heartbeat) perfectly healthy with led2
    //     and led3 dark.  That is the reported symptom.
    //
    //     WHAT IT MEASURES -- on the RAW ch* values from mic_capture, so a
    //     broken filter cannot hide a live microphone (or fake a dead one):
    //       ch_act[k] : this channel's sample moved more than ACT_TH since
    //                   the previous audio frame, held for 0.5 s.  A stuck
    //                   data line, an unpowered microphone or a missing bit
    //                   clock reads 0 forever.
    //       ch_lvl[k] : log2 of the running mean of |sample| = an ABSOLUTE
    //                   6 dB-granular level meter per channel, reference
    //                   included.  0 = flat zero, ~9 = a live MEMS noise
    //                   floor, 17+ = loud signal.
    //
    //     Both update once per audio frame (ch_stb_clk, every 1024 clk), so
    //     the monitor is a handful of adders running at 48.8 kHz with no
    //     impact on the 50 MHz timing.
    //
    //     Read it over the serial link (see the status line in
    //     mic_uart_framer) and on led1/led2 (see the LED table).
    //==================================================================
    localparam integer ACT_TH = 512;             // 24-bit LSBs: above a stuck
                                                 // line, under a MEMS floor

    wire [6:0]  ch_act;                          // per-channel "data moves"
    wire [34:0] ch_lvl;                          // 7 x 5-bit level codes

    // The CENTRE REFERENCE is the channel that matters: every correlation
    // is computed against it, so a dead reference stops the whole chain
    // while lanes 0..2 can look perfectly healthy.  "any channel moves"
    // could not tell those two apart (2026-10-14, after a board reported
    // led1 solid + led2/led3 dark).  The per-channel detail is still on the
    // serial status line's D= field.
    wire        ref_alive = ch_act[6];           // reference (ch6) is moving

    audio_ch_mon #(.DATA_W(DATA_W), .ACT_TH(ACT_TH)) u_mon0
        (.clk(clk_g), .rst_n(rst_n_clk), .stb(ch_stb_clk),
         .x(ch0), .act(ch_act[0]), .lvl(ch_lvl[4:0]));
    audio_ch_mon #(.DATA_W(DATA_W), .ACT_TH(ACT_TH)) u_mon1
        (.clk(clk_g), .rst_n(rst_n_clk), .stb(ch_stb_clk),
         .x(ch1), .act(ch_act[1]), .lvl(ch_lvl[9:5]));
    audio_ch_mon #(.DATA_W(DATA_W), .ACT_TH(ACT_TH)) u_mon2
        (.clk(clk_g), .rst_n(rst_n_clk), .stb(ch_stb_clk),
         .x(ch2), .act(ch_act[2]), .lvl(ch_lvl[14:10]));
    audio_ch_mon #(.DATA_W(DATA_W), .ACT_TH(ACT_TH)) u_mon3
        (.clk(clk_g), .rst_n(rst_n_clk), .stb(ch_stb_clk),
         .x(ch3), .act(ch_act[3]), .lvl(ch_lvl[19:15]));
    audio_ch_mon #(.DATA_W(DATA_W), .ACT_TH(ACT_TH)) u_mon4
        (.clk(clk_g), .rst_n(rst_n_clk), .stb(ch_stb_clk),
         .x(ch4), .act(ch_act[4]), .lvl(ch_lvl[24:20]));
    audio_ch_mon #(.DATA_W(DATA_W), .ACT_TH(ACT_TH)) u_mon5
        (.clk(clk_g), .rst_n(rst_n_clk), .stb(ch_stb_clk),
         .x(ch5), .act(ch_act[5]), .lvl(ch_lvl[29:25]));
    // u_mon6 is the one that matters most: the CENTRE reference.
    audio_ch_mon #(.DATA_W(DATA_W), .ACT_TH(ACT_TH)) u_mon6
        (.clk(clk_g), .rst_n(rst_n_clk), .stb(ch_stb_clk),
         .x(ch6), .act(ch_act[6]), .lvl(ch_lvl[34:30]));

    //------------------------------------------------------------------
    // u_mon7 -- the OTHER lane-3 phase (added 2026-10-15).
    //
    //  Without this the board cannot distinguish "the centre microphone is
    //  strapped to the other WS phase" from "the whole array is dead": both
    //  show up as ref_alive = 0, and every correlation is 0 either way, so
    //  leds[2]/leds[3] stay dark in both cases.
    //
    //  With it, the two are one glance apart:
    //      leds[1] blinking + leds[3] LIT   -> the centre mic lives in the
    //                                          phase MAP6 did not pick: set
    //                                          MAP6 to the other value
    //      leds[1] blinking + leds[3] DARK  -> neither lane-3 phase answers:
    //                                          supply / CHIPEN / wiring
    //------------------------------------------------------------------
    wire slot_other_alive;

    audio_ch_mon #(.DATA_W(DATA_W), .ACT_TH(ACT_TH)) u_mon7
        (.clk(clk_g), .rst_n(rst_n_clk), .stb(ch_stb_clk),
         .x(slot_l3_other), .act(slot_other_alive),
         // .lvl deliberately left unconnected: only "does this phase move at
         // all" is needed here, and leaving it open lets the tool trim the
         // level meter out of this instance instead of computing a wire
         // nobody reads.  The centre microphone's level is on the UART
         // status line's E= field.
         .lvl());

    // any of the six RING microphones (lanes 0..2) delivering data.  Used by
    // the lamps to separate "only the reference is missing" from "nothing at
    // all is arriving" -- see the LED output stage table.
    wire ring_alive = |ch_act[5:0];

    //==================================================================
    // 4.  UART packet framing  (clk domain)
    //==================================================================
    wire        uart_out;
    wire [15:0] dropped;
    wire [7:0]  bytes_sent;

    // doa_done lives here rather than next to the solver because the framer
    // below already needs it as the diagnostic strobe.
    wire        doa_done;

    //------------------------------------------------------------------
    // DOA diagnostic payload, used when UART_PKT_MODE = 1.
    //
    // Declared HERE (before the framer that reads them) and driven in
    // section 5c, where the DOA results exist.  See that section for why
    // they are combinational.
    //------------------------------------------------------------------
    wire [(16*6)-1:0]   diag_tau;    // 6 x signed Q8 samples
    wire signed [15:0]  diag_bear;   // Q1.14, 16384 == pi/2 == 90 deg (as
                                     // mic_uart_framer.v / doa_diag.py define it)
    wire [7:0]          diag_qual;   // 0..255 fit quality
    wire [7:0]          diag_resid;  // Q8 samples, largest residual
    wire [3:0]          diag_sect;
    wire [5:0]          diag_valid;  // per-microphone valid mask
    wire [(3*6)-1:0]    diag_lvl;    // per-microphone peak level codes (6 dB)
    // Declared here because the framer instance below consumes it, but
    // assigned in section 5b-2 where ref_energy finally exists.
    wire [3:0]          ref_e_code;  // log2(ref_energy), the gate's own gauge
    // Same reason: consumed by the framer instance, assigned in section 5c
    // where `bearing` exists.  Declaring at the point of assignment made the
    // instance's use an implicit 1-bit wire and the later declaration then
    // collided with it (vlog-2388).
    wire [8:0]          bear_deg;    // bearing in whole degrees, 0..359

    //------------------------------------------------------------------
    //  Ring-report plumbing (see the framer's rpt_* ports).
    //
    //  Declared up here, before the framer instance, because Verilog needs a
    //  name declared before it is USED: connecting the instance to an
    //  undeclared identifier creates an implicit 1-bit wire and then the real
    //  declaration collides with it (vlog-2388, already bitten once on
    //  bear_deg).  The drivers are down in sections 5c/7, which is fine.
    //------------------------------------------------------------------
    //  rpt_mic is a WIRE now, not a register (2026-11-01).  It used to be
    //  latched from a raw-TDOA argmax on every solved frame, while rpt_sect
    //  came from the sector the ring LATCHED on the frames the solver
    //  accepted -- two different frames, so the line could say M=3 next to a
    //  LED group belonging to mic4.  Both are now derived from sect_hold
    //  alone; see "DISPLAY MIC" in section 5d.
    wire [2:0]               rpt_mic;     // 0..5, mic the lit group is on
    wire [3:0]               rpt_sect;    // = sect_q, the sector on show
    wire                     ring_lit;    // from u_ring (A line's G= field)

    //  Display view of sect_hold: sect_q is sect_hold re-centred on a
    //  microphone axis, mic_show is that microphone.  Declared here because
    //  the framer instance below connects to them; driven in section 5d.
    wire [3:0]               sect_q;      // sector whose centre is a mic axis
    wire [2:0]               mic_show;    // that mic, 0..5

    //  The ring's own state, declared HERE for the reason in the note above:
    //  the framer's rpt_stb / rpt_sect ports are driven by dir_update and
    //  sect_hold, which used to be declared down in section 6 -- connecting
    //  the instance to them first would have created implicit 1-bit wires.
    reg  [3:0]               sect_hold;   // sector the ring is showing
    reg                      dir_update;  // 1-clk ring refresh (0.5 s throttle)

    //------------------------------------------------------------------
    //  First-arrival event plumbing (see EVT_GAP_MS and first_arrival.v).
    //
    //  Declared BEFORE the framer instance for the same reason as rpt_*: an
    //  identifier used before its declaration becomes an implicit 1-bit wire
    //  and the real declaration then collides (vlog-2388, already bitten once
    //  on bear_deg).  The driver is in section 5d-2.
    //------------------------------------------------------------------
    wire                     ev_stb;      // 1-clk pulse, once per sound event
    wire [2:0]               ev_mic;      // 0..5 ring, 6 = centre heard it first
    wire signed [15:0]       ev_tau;      // that mic's RAW TDOA, Q8 samples
    wire [3:0]               ev_l;        // the three LEDs facing that mic,
    wire [3:0]               ev_c;        //   in PHYSICAL strip order
    wire [3:0]               ev_r;


    //------------------------------------------------------------------
    //  The bearing that the UART actually publishes.
    //
    //  NOT the live `bearing` net.  `diag_stb` is doa_done, which pulses on
    //  EVERY solved frame -- including the ones the robust fit threw away
    //  (board capture 2026-10-15: Q = 00 with R = 00, ninl = 0, on roughly
    //  half the frames).  Reporting those means half the B= samples in a
    //  calibration run are pure noise, and `calib.py` has no way to tell
    //  which: B= rides the status line and Q= rides a different line, so
    //  they cannot be paired up reliably.
    //
    //  So B=/diag_bear now report the last bearing the solver was willing to
    //  stand behind -- the same condition that drives the ring, which makes
    //  a checkable invariant: a rejected frame must NOT move B=.
    //      bearing  raw, live, every frame   (kept for the debug latch)
    //      bear_lat latched on sol_valid && snd_now, published
    //------------------------------------------------------------------
    reg signed [17:0]   bear_lat;

    mic_uart_framer #(
        .DATA_W  (DATA_W),
        .PKT_MODE(UART_PKT_MODE),
        .LED_REVERSE(LED_REVERSE)
    ) u_framer (
        .clk      (clk_g),
        .rst_n    (rst_n_clk),
        .ch_stb   (ch_stb_clk),
        .ch0(ch0), .ch1(ch1), .ch2(ch2), .ch3(ch3),
        .ch4(ch4), .ch5(ch5), .ch6(ch6),
        .frame_cnt(frame_cnt),
        .ch_act   (ch_act),
        .ch_lvl   (ch_lvl),
        .diag_stb (doa_done),
        .diag_tau (diag_tau),
        .diag_bear(diag_bear),
        .diag_qual(diag_qual),
        .diag_sect(diag_sect),
        .diag_resid(diag_resid),
        .diag_valid(diag_valid),
        .diag_lvl (diag_lvl),
        .diag_energy(ref_e_code),
        .diag_deg(bear_deg),
        // The human line is printed on the ring's refresh strobe, so the log
        // and the lamps change together (2 Hz).  rpt_tau is gone: the raw Q8
        // tau was one of the values that line exists to replace.  rpt_lit
        // stays because the "A" event line still prints G=<0|1>.
        .rpt_stb (dir_update),
        .rpt_mic (rpt_mic),
        .rpt_sect(rpt_sect),
        .rpt_lit (ring_lit),
        .ev_stb  (ev_stb),
        .ev_mic  (ev_mic),
        .ev_tau  (ev_tau),
        .ev_l    (ev_l),
        .ev_c    (ev_c),
        .ev_r    (ev_r),
        .tx       (uart_out),
        .dropped  (dropped),
        .bytes_sent(bytes_sent)
    );

    assign uart_txp = uart_out;

    //==================================================================
    // 5.  Direction of arrival: TDOA engine -> low-pass -> robust solver.
    //
    //     All three blocks run in the clk (50 MHz) domain and are fed from
    //     the CONDITIONED channel registers (fch0..fch6: band-passed and
    //     pre-emphasised in mic_capture), with ch_stb_clk as the sample-valid
    //     strobe.  mic_capture holds each sample set stable until the next
    //     frame arrives, which is far longer than the chain needs, so no FIFO
    //     is required.
    //
    //     The centre microphone (fch6) is the reference; the six ring mics
    //     are fch0..fch5.  Feeding the engine the filtered channels is only
    //     legitimate because the reference goes through the SAME chain: a
    //     filter common to both inputs of a correlation cannot move its peak,
    //     so the TDOA survives exactly.
    //==================================================================
    wire                     tdoa_done;
    wire [4*6-1:0]           tdoa_lag;
    wire [(FRAC_W+1)*6-1:0]  tdoa_frac;
    wire [ACC_W*6-1:0]       tdoa_mag;
    wire [5:0]               tdoa_valid;      // per-mic real-peak flags
    wire                     tdoa_any_valid;
    wire signed [ACC_W-1:0]  ref_energy;

    tdoa_engine #(
        .DATA_W (DATA_W),
        .N      (N),
        .L      (LAG_RANGE),
        .ACC_W  (ACC_W),
        .SHIFT  (MAC_SHIFT),
        .FRAC_W (FRAC_W),
        .NMIC   (6),
        .NBAND  (NBAND)
    ) u_tdoa (
        .clk        (clk_g),
        .rst_n      (rst_n_clk),
        .valid      (ch_stb_clk),
        .ref_full   (fch6),
        .ref_b      (ref_b),
        .mic_b      (mic_b),
        .done       (tdoa_done),
        .lag_flat   (tdoa_lag),
        .frac_flat  (tdoa_frac),
        .mag_flat   (tdoa_mag),
        .valid_flat (tdoa_valid),
        .any_valid  (tdoa_any_valid),
        .ref_energy (ref_energy)
    );

    //------------------------------------------------------------------
    // 5a. PEAK-MAGNITUDE GUARD, PIPELINED  (added 2026-10-11;
    //                                       restructured 2026-10-11)
    //
    //     valid_flat only says "this microphone produced a POSITIVE
    //     correlation peak" -- and noise produces positive peaks too.  A
    //     weak, acoustically blocked, mis-mapped or unclocked channel can
    //     therefore still be flagged valid, drag the FIRST least-squares
    //     pass away from the truth, and the robust pass -- which measures
    //     every residual against that poisoned fit -- then ends up
    //     discarding the healthy microphones instead.  The guard has to
    //     act BEFORE the fit, not after it.
    //
    //     A microphone whose peak magnitude is more than MAG_MIN_SH
    //     halvings below the strongest one is dropped for this pass.
    //     MAG_MIN_SH >= 16 disables the check.
    //
    //     The peak magnitude mixes level and coherence, which is exactly
    //     what is wanted here: the question is only "is this microphone
    //     hearing what the others hear".  The same 6 dB steps travel in the
    //     diagnostic packet, so a persistently weak channel is visible on
    //     the PC side without any extra instrumentation.
    //
    //     WHY THIS IS NOW THREE REGISTERED STAGES
    //     ---------------------------------------
    //     The first version of the guard was ONE flat combinational cone:
    //         mag_top   -- 6 x 36-bit priority compares, 6 deep
    //       -> mag_floor -- a 36-bit shift
    //       -> valid_eff -- 6 x 36-bit compares, then a 6-way AND
    //     Place & route measured that cone at ~32 ns against a 20 ns clock
    //     and it was the SOLE cause of the -9.055 ns setup violation that
    //     capped this design at ~34 MHz (see
    //     baseline_2026-10-11/audio.timing_paths, where all six worst paths
    //     ran u_tdoa/valid_flat_* -> u_tdoa_lpf/out_valid_*).
    //
    //     It is now three stages, each holding at most a pair of compares:
    //       stage 1 : three independent pairwise maxima of (0,1) (2,3) (4,5)
    //       stage 2 : reduce those three to the overall maximum
    //       stage 3 : compare each microphone against the floor
    //     The floor itself is free: `>>> MAG_MIN_SH` on a REGISTERED value is
    //     pure wiring, not logic.
    //
    //     NO EXTRA STROBE LOGIC IS NEEDED IN THE DATA PATH.  The
    //     {lag, frac, mag, valid} bundle is held stable by tdoa_engine for a
    //     whole pass -- ~5.6 ms, i.e. hundreds of thousands of clk -- so the
    //     pipeline only has to carry the STROBE.  tdoa_done is delayed by
    //     exactly the three stages (see `strobes`) and handed to tdoa_filter
    //     as in_done, so the filter still samples valid_eff_r and its strobe
    //     in the SAME cycle, which is the contract tdoa_filter documents.
    //
    //     Latency added: 3 clk.  Cost: 3 x ACC_W + ACC_W + 6 FF ~ 150 FF
    //     against a 6% register utilisation.  Cheapest fix in the file.
    //------------------------------------------------------------------
    wire [ACC_W-1:0] gm0 = tdoa_valid[0] ? tdoa_mag[ACC_W*0 +: ACC_W] : {ACC_W{1'b0}};
    wire [ACC_W-1:0] gm1 = tdoa_valid[1] ? tdoa_mag[ACC_W*1 +: ACC_W] : {ACC_W{1'b0}};
    wire [ACC_W-1:0] gm2 = tdoa_valid[2] ? tdoa_mag[ACC_W*2 +: ACC_W] : {ACC_W{1'b0}};
    wire [ACC_W-1:0] gm3 = tdoa_valid[3] ? tdoa_mag[ACC_W*3 +: ACC_W] : {ACC_W{1'b0}};
    wire [ACC_W-1:0] gm4 = tdoa_valid[4] ? tdoa_mag[ACC_W*4 +: ACC_W] : {ACC_W{1'b0}};
    wire [ACC_W-1:0] gm5 = tdoa_valid[5] ? tdoa_mag[ACC_W*5 +: ACC_W] : {ACC_W{1'b0}};

    // Strobe pipeline.  strobes[0] is tdoa_done one clk late, strobes[1] two
    // clks late, strobes[2] three clks late -- and strobes[2] is EXACTLY the
    // cycle in which valid_eff_r is first valid, which is why it, and not
    // tdoa_done, drives tdoa_filter.
    reg [2:0] strobes;
    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) strobes <= 3'd0;
        else            strobes <= {strobes[1:0], tdoa_done};
    end

    // stage 1: three independent pairwise maxima, captured on tdoa_done
    reg [ACC_W-1:0] mg_p01, mg_p23, mg_p45;
    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            mg_p01 <= {ACC_W{1'b0}};
            mg_p23 <= {ACC_W{1'b0}};
            mg_p45 <= {ACC_W{1'b0}};
        end else if (tdoa_done) begin
            mg_p01 <= (gm0 > gm1) ? gm0 : gm1;
            mg_p23 <= (gm2 > gm3) ? gm2 : gm3;
            mg_p45 <= (gm4 > gm5) ? gm4 : gm5;
        end
    end

    // stage 2: the overall maximum.  mg_p01/mg_p23 are compared one clk
    // after they were captured, i.e. on strobes[0].
    wire [ACC_W-1:0] mg_lo = (mg_p01 > mg_p23) ? mg_p01 : mg_p23;
    reg  [ACC_W-1:0] mag_top_r;
    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk)      mag_top_r <= {ACC_W{1'b0}};
        else if (strobes[0]) mag_top_r <= (mg_lo > mg_p45) ? mg_lo : mg_p45;
    end

    // stage 3: threshold every microphone against the (registered) floor.
    wire [ACC_W-1:0] mag_thr = (MAG_MIN_SH >= 16) ? {ACC_W{1'b0}}
                                                  : (mag_top_r >>> MAG_MIN_SH);

    reg [5:0] valid_eff_r;
    integer   mzi;
    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            valid_eff_r <= 6'd0;
        end else if (strobes[1]) begin
            for (mzi = 0; mzi < 6; mzi = mzi + 1)
                valid_eff_r[mzi] <= tdoa_valid[mzi] &&
                                    (tdoa_mag[ACC_W*mzi +: ACC_W] >= mag_thr);
        end
    end

    //------------------------------------------------------------------
    // 5b. TDOA LOW-PASS  (added 2026-10-11)
    //
    //     One noisy (lag, frac) pair per microphone per pass is what makes
    //     the bearing -- and with it the LED ring -- jitter, and what
    //     occasionally throws the estimate a whole sector away.  The
    //     filter averages the fractional TDOA of every microphone with a
    //     forgetting factor before the solver sees it.  The module sits at
    //     the end of tdoa_engine.v (same TDOA front end, same {lag, frac}
    //     contract); see its header for the arithmetic, and TDOA_LPF_SH for
    //     the tuning (0 = bypass).  The per-mic valid flags travel with it,
    //     so the solver always sees flags and TDOAs from the SAME pass.
    //
    //     in_done is the guard's strobe, NOT tdoa_done: the magnitude guard
    //     in section 5a is now three registered stages deep, so its
    //     valid_eff_r output appears three clk after tdoa_done.  Using
    //     strobes[2] here is what keeps strobe and flags in the same cycle.
    //------------------------------------------------------------------
    wire                     tdoa_done_f;
    wire [4*6-1:0]           tdoa_lag_f;
    wire [(FRAC_W+1)*6-1:0]  tdoa_frac_f;
    wire [5:0]               tdoa_valid_f;
    wire                     tdoa_any_valid_f;

    tdoa_filter #(
        .NMIC   (6),
        .FRAC_W (FRAC_W),
        .LAG_W  (4),
        .SH     (TDOA_LPF_SH),
        .SLEW   (TDOA_SLEW)
    ) u_tdoa_lpf (
        .clk          (clk_g),
        .rst_n        (rst_n_clk),
        .in_done      (strobes[2]),
        .in_lag       (tdoa_lag),
        .in_frac      (tdoa_frac),
        .in_valid     (valid_eff_r),
        .in_any_valid (tdoa_any_valid),
        .out_done     (tdoa_done_f),
        .out_lag      (tdoa_lag_f),
        .out_frac     (tdoa_frac_f),
        .out_valid    (tdoa_valid_f),
        .out_any_valid(tdoa_any_valid_f)
    );

    //==================================================================
    // 5b-2.  "IS THERE ANY SOUND AT ALL" -- THE MISSING MASTER GATE
    //
    //  Before 2026-10-15 this energy test existed but drove ONLY the LED
    //  ring, so the rest of the chain ran unconditionally.  Board capture
    //  (PKT_MODE = 2, silent room) showed why that is wrong:
    //
    //      D=7F     all seven channels moving
    //      E=0F100F0E0D0F11   level codes 13..17, i.e. ~72..84 dB SPL
    //      V=3F     all six correlations "valid"
    //      Q=00 R=00  on about HALF the frames -> ninl = 0, EVERY
    //                 microphone rejected: those frames contain no plane
    //                 wave at all, only room noise that happens to
    //                 correlate, and
    //      Q=FF R=12  on the rest -> a perfectly good solution
    //
    //  So the engine was reporting "the microphones agree" (led2/led3 lit)
    //  and the ring was chasing the sector of frames the solver itself had
    //  rejected.  The gate below is what makes "has the reference
    //  microphone heard anything" a precondition for all of it.
    //
    //  ANY_SND_E is the threshold, in the same units as ref_energy
    //  (ref_energy = RMS(x16)^2 with x16 = the 24-bit sample / 256).  The
    //  calibration, so this is tunable without guessing:
    //      E= code k  =>  mean|sample| = 2^k LSB(24)  =>  ~k*6 - 30 dB SPL
    //      code  9  ->  ~24 dB SPL   (a bare MEMS noise floor)
    //      code 13  ->  ~48 dB SPL   (a quiet room)
    //      code 17  ->  ~72 dB SPL   (talking / a noisy room)
    //      ANY_SND_E = 1024  <=>  x16 rms = 32  <=>  ~60 dB SPL
    //  Set ANY_SND_E = 0 to disable the gate completely (it is a parameter,
    //  so the escape hatch is free and always available).
    //==================================================================
    wire snd_now = (ANY_SND_E == 0) ? 1'b1 : (ref_energy > ANY_SND_E);

    // ---- the gate's OWN number, for the UART status line ---------------
    //
    // log2(ref_energy), saturated to 15.  Printed as W=<hex> so the board
    // can report what the ENGINE believes, which is the only way to explain
    // "leds[2] dark": the E= field on that same line measures the RAW
    // sample, and between the two sits the whole mic_cond chain (10 dB on
    // white noise, 20..30 dB across speech).  A healthy-looking E=17 with a
    // tiny W= means the raw energy is low-frequency and the chain removed
    // it -- which is the chain doing its job, not a microphone fault.
    //
    // ref_energy is signed, so a negative (impossible in practice) value
    // falls through every compare to 0.
    assign ref_e_code = (ref_energy >= 32768) ? 4'd15
                      : (ref_energy >= 16384) ? 4'd14
                          : (ref_energy >=  8192) ? 4'd13
                          : (ref_energy >=  4096) ? 4'd12
                          : (ref_energy >=  2048) ? 4'd11
                          : (ref_energy >=  1024) ? 4'd10
                          : (ref_energy >=   512) ? 4'd9
                          : (ref_energy >=   256) ? 4'd8
                          : (ref_energy >=   128) ? 4'd7
                          : (ref_energy >=    64) ? 4'd6
                          : (ref_energy >=    32) ? 4'd5
                          : (ref_energy >=    16) ? 4'd4
                          : (ref_energy >=     8) ? 4'd3
                          : (ref_energy >=     4) ? 4'd2
                          : (ref_energy >=     2) ? 4'd1
                          :                         4'd0;

    // Everything downstream sees "a believable peak AND someone is talking":
    // the solver start condition and leds[2] both take THIS, so they can no
    // longer disagree with each other or with the ring.
    wire any_valid_g = tdoa_any_valid_f && snd_now;

    // doa_done is declared in section 4, next to the diagnostic payload the
    // UART framer latches on this strobe.
    wire signed [17:0] bearing;
    wire [3:0]        sector;
    wire [23:0]       confidence;      // 0..255 quality, larger = better
    wire [15:0]       doa_resid;       // Q(FRAC_W) samples

    // ---- bearing in whole degrees, 0..359, for the UART status line -----
    //
    // Added 2026-10-16 alongside the continuous SECT_OFF_DEG: calibrating a
    // rotation offset needs an ANGLE, and the 12-bin S= field has only 30 deg
    // of resolution -- enough to see that something is wrong, not enough to
    // measure by how much.
    //
    // bearing is Q1.16 fraction of pi (65536 == 180 deg), signed, -pi..+pi:
    //     deg = (bearing + 65536) * 180 / 65536 = (bearing + 65536) * 45 >> 14
    // +180 and -180 are the same direction, so the +pi edge folds back to 0.
    //
    // NOTE the +65536: B=180 means "theta = 0" in the microphone frame, i.e.
    // B = theta + 180 (mod 360).  That offset is deliberate and it cancels
    // the +pi inside doa_solver's sector quantiser
    //     sector = floor((theta + SECT_OFF_DEG + pi) / (2pi) * NSEC)
    // which is exactly why a calibration can use the simple
    //     SECT_OFF_DEG = wrap180(true_angle - B)
    // Verified numerically against the RTL integer expressions (both the
    // 65536*45>>14 above and the quantiser) over every 5 deg of theta and
    // 16 array rotations.
    wire signed [19:0] bear_ext = {{2{bear_lat[17]}}, bear_lat};  // to 20 bits
    wire        [19:0] bear_abs = bear_ext + 20'sd65536;         // 0..131072

    //  PIPELINE (2026-11-01).  bear_lat -> +65536 -> *45 -> >>14 -> clamp was
    //  ONE combinational cone.  It measured +1.067 ns of slack before the
    //  polarity fix and became the worst setup path of the whole design after
    //  it re-arranged the placement (+0.035 ns, i.e. no margin at all).  Both
    //  ends of the cone are registers already, so one 20-bit register after
    //  the adder -- one clock, 20 ns, on a value that only changes when a new
    //  bearing is ACCEPTED -- halves it and cannot be observed downstream:
    //  bear_deg was already a frame stale (bear_lat updates on the same
    //  doa_done edge the framer samples on).
    reg [19:0] bear_abs_q;
    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) bear_abs_q <= 20'd65536;   // == bear_lat 0 -> deg 180
        else            bear_abs_q <= bear_abs;
    end

    wire        [25:0] bear_mul = bear_abs_q * 20'd45;           // <= 5898240

    // 9 bits are enough AND exact: bear_mul <= 5898240, so bear_mul[25:14]
    // never exceeds 360 and its top three bits are always zero.
    //
    // The previous form compared the 12-bit slice [25:14] and assigned it to
    // the 9-bit bear_deg, which synthesis reported as
    //     WARN (EX3791) Expression size 12 truncated to fit in target size 9
    // i.e. the silent-narrowing class this project keeps finding.  Every
    // operand here is explicitly 9 bits, including the out-of-range clamp, so
    // there is nothing left for the synthesiser to narrow.
    wire [8:0] bd_raw = (bear_mul[25:23] != 3'd0) ? 9'd511 : bear_mul[22:14];
    assign bear_deg = (bd_raw >= 9'd360) ? 9'd0 : bd_raw;

    doa_solver #(
        .NMIC   (6),
        .FRAC_W (FRAC_W),
        .VEC_W  (24),
        .NSEC   (NSEC),
        .SECT_OFF_DEG(SECT_OFF_DEG),
        .MIRROR_X(MIRROR_X),
        .FLIP_180(BEARING_FLIP),
        .TRIM0  (TRIM0),
        .TRIM1  (TRIM1),
        .TRIM2  (TRIM2),
        .TRIM3  (TRIM3),
        .TRIM4  (TRIM4),
        .TRIM5  (TRIM5),
        .R_MAX  (DOA_R_MAX),
        .R_SH   (DOA_R_SH)
    ) u_doa (
        .clk       (clk_g),
        .rst_n     (rst_n_clk),
        .tdoa_done (tdoa_done_f),
        .lag_flat  (tdoa_lag_f),
        .frac_flat (tdoa_frac_f),
        .valid_flat(tdoa_valid_f),
        .any_valid (any_valid_g),        // <- energy-gated, see 5b-2
        .done      (doa_done),
        .bearing   (bearing),
        .sector    (sector),
        .confidence(confidence),
        .resid_max (doa_resid)
    );

    //------------------------------------------------------------------
    // [新增] DOA 结果外引。
    //   必须写在 u_doa 例化**之后**：bearing/sector/confidence/doa_done 是
    //   generate 作用域内的局部线网，若把 assign 放到模块末尾会报 not declared。
    //   doa_valid_o 用 doa_done（解算完成那一拍），置信度门控交给视频侧
    //   pan_ctrl 的 CONF_MIN 自行判断 —— 两侧门限可独立调，互不影响。
    //------------------------------------------------------------------
    assign doa_bearing_o = bearing;
    assign doa_sector_o  = sector;
    assign doa_conf_o    = confidence[7:0];
    assign doa_valid_o   = doa_done;

    //==================================================================
    // 5c. DOA diagnostic payload  (the wires are declared in section 4)
    //
    //     Deliberately combinational.  doa_done is BOTH the strobe the framer
    //     latches on AND the cycle in which the solver has already published
    //     the new bearing / quality, so registering these here would report
    //     the previous pass.
    //
    //     tau_m is rebuilt from the FILTERED lag/frac (the values the solver
    //     actually used, which is what a calibration must null) as signed Q8
    //     samples: one sample is 10.2 degrees of bearing at R = 40mm and
    //     48.83kHz, so Q8 resolves 0.04 degrees -- far finer than the array
    //     can deliver, which is the point of a diagnostic link.
    //==================================================================
    wire [15:0] tau8 [0:5];

    genvar gm;
    generate
        for (gm = 0; gm < 6; gm = gm + 1) begin : g_tau8
            wire signed [3:0]      lg = $signed(tdoa_lag_f[4*gm +: 4]);
            wire signed [FRAC_W:0] fr = $signed(tdoa_frac_f[(FRAC_W+1)*gm +: (FRAC_W+1)]);
            wire signed [31:0]     tq = ({{28{lg[3]}}, lg} <<< FRAC_W)
                                      + {{(31-FRAC_W){fr[FRAC_W]}}, fr};

            // Q(FRAC_W) -> Q8 with round-to-nearest; FRAC_W must be >= 9
            wire signed [31:0] tq8 = (tq + (32'sd1 <<< (FRAC_W-9))) >>> (FRAC_W-8);

            // PHYSICAL BOUND: |tau| <= R/c = 116.6 us = 5.7 samples, so in Q8
            // the value can never leave +-1536.  The clamp below therefore
            // never actually bites -- it exists so the 32 -> 16 bit narrowing
            // is EXPLICIT, which is what silences the synthesis warning
            //     WARN (EX3791) : Expression size 30 truncated to fit 16
            // that this assignment used to raise (impl/gwsynthesis/audio.log).
            assign tau8[gm] = (tq8 >  32'sd32767) ? 16'sh7FFF :
                              (tq8 < -32'sd32768) ? 16'sh8000 :
                                                    tq8[15:0];
        end
    endgenerate

    // CHANNEL ORDER -- corrected 2026-10-15.
    //
    // The framer packs the packet from the LOW end of diag_tau
    //     mic_uart_framer.v: pkt[4 + 2*i] <= diag_tau[16*i + 15 -: 8];
    // so diag_tau[15:0] becomes the FIRST transmitted tau pair.  The packet
    // header ("bytes 4..15 : tau0..tau5") and tools/doa_diag.py
    // (`_s16(p, 4 + 2*m)`) both read that first pair as tau0, so tau0 has to
    // be the RIGHTMOST concatenation operand.
    //
    // This used to be { tau8[0], ..., tau8[5] }, i.e. tau0 in the TOP bits,
    // which actually transmitted tau5,tau4,tau3,tau2,tau1,tau0.  The
    // symptom is silent: doa_diag.py --ref-theta prints a MIRRORED TRIM
    // table (mic0<->mic5, mic1<->mic4, mic2<->mic3) and filling it into
    // TRIM0..TRIM5 calibrates every microphone with another one's error.
    // diag_lvl just below is the reference for the convention used here:
    // mic0 sits in the LOW bits.
    assign diag_tau   = { tau8[5], tau8[4], tau8[3], tau8[2], tau8[1], tau8[0] };

    //  The FIRST-MIC ARGMAX INPUT REGISTER that used to sit here is gone
    //  (2026-11-01), together with the argmax it fed.  It existed only to
    //  split tau -> argmax -> UART across two clocks; there is no argmax any
    //  more (see "DISPLAY MIC" below), so it had a single dead consumer.
    //  diag_tau itself is unchanged and still feeds the diagnostic packet.

    //==================================================================
    // 5d.  DISPLAY MIC FOR THE HUMAN LINE  (derived from the ring's sector)
    //
    //  WHAT THIS USED TO BE, AND WHY IT WAS WRONG
    //
    //  Until 2026-11-01 the M= field was the argmax of the six raw TDOAs --
    //  "which microphone's correlation peak came first".  The LED numbers on
    //  the same line came from `sect_hold`, the sector the ring LATCHED on the
    //  frames the robust fit accepted.  Two different quantities, latched at
    //  two different times:
    //
    //      ring   : 6-channel least-squares bearing, held for DIR_MIN_MS
    //      M=     : argmax of the LIVE per-frame TDOA vector, free-running --
    //               it also moved on frames the fit had just thrown away
    //
    //  so they could describe different frames, and on the board they did.
    //  Capture of 2026-11-01, talker in front of mic3:
    //      ANG=068deg  LED=06,07,08  M=3  Q=091
    //      ANG=045deg  LED=07,08,09  M=3  Q=094
    //      ANG=049deg  LED=07,08,09  M=2  Q=104     <- M=2 with LED 7,8,9
    //  M cannot be 2 while the bearing says mic4's side, and it cannot be 3
    //  while the lit group is centred on 7 (half-way between mic3 and mic4).
    //  Three numbers on one line that a human is supposed to read as ONE
    //  statement.
    //
    //  WHAT IT IS NOW
    //
    //  One source of truth: `sect_hold`, the sector the ring is showing.
    //      mic k  <=>  M=k  <=>  LEDs {2k-1, 2k, 2k+1}
    //  holds by construction at every angle, because both fields are read off
    //  the same register.  The old guard against an unsigned compare blaming
    //  one microphone forever is obsolete: there is no compare here any more.
    //
    //  Index arithmetic.  The sector index runs COUNTER-clockwise, one step
    //  per 30 deg, while the microphones are numbered CLOCKWISE at 60 deg, so
    //      mic = (NSEC-1 - sector) / 2
    //  after folding in LED_REVERSE.  `sect_q` then re-centres the displayed
    //  sector on that microphone's axis.  This is also why the lit group is
    //  always {2k-1,2k,2k+1} and no longer slides half a step: a ring that
    //  shows a direction which belongs to no microphone cannot agree with the
    //  microphone number printed next to it, and the printed table is
    //      mic0 -> 11,0,1   mic1 -> 1,2,3   mic2 -> 3,4,5
    //      mic3 -> 5,6,7    mic4 -> 7,8,9   mic5 -> 9,10,11
    //  i.e. exactly the 6 values this produces.
    //
    //  Cost: the ring now steps 60 deg (2 LEDs) instead of 30 deg.  That is
    //  what the microphones support -- six of them, 60 deg apart -- and the
    //  fine angle is still on the line as ANG=<deg>.  Recovery if the group is
    //  consistently one microphone off: SECT_OFF_DEG +=-60 rotates the picture
    //  by exactly one microphone, which is the natural granularity here.
    //==================================================================
    localparam [3:0] NSEC_M1  = 4'd11;      // NSEC-1; NSEC is 12 everywhere

    wire [3:0]       sect_rev = LED_REVERSE ? (NSEC_M1 - sect_hold) : sect_hold;
    wire [3:0]       led_c    = {mic_show, 1'b0};   // 2*mic = the LED centre

    assign mic_show = sect_rev[3:1];        // 0..5, = floor(sect_rev / 2)
    assign sect_q   = LED_REVERSE ? (NSEC_M1 - led_c) : led_c;
    assign rpt_mic  = mic_show;
    //------------------------------------------------------------------
    // END OF SECTION 5d.
    //
    // The 3-level comparison tree and its register that used to follow are
    // deleted (2026-11-01).  They computed the argmax of the six raw TDOAs,
    // which is a "who was first THIS frame" answer -- unlatched, and taken
    // from frames the solver rejected.  The human line needs an answer that
    // matches the LEDs it prints beside it, so M= is now a view of the sector
    // the ring is showing.  The raw per-microphone TDOAs are unchanged and
    // still shipped in the diagnostic packet (PKT_MODE = 1), which is where
    // they belong: a calibration channel, not a 0.5 s display line.
    //------------------------------------------------------------------

    //==================================================================
    // 5d-2.  FIRST-ARRIVAL EVENT  (2026-11-01)
    //
    //  The human line above is a SNAPSHOT (one per displayed direction): it
    //  repeats the same conclusion
    //  while nothing changes, so it cannot say "this is the microphone that
    //  heard the sound start".  This block answers exactly that, once per
    //  utterance, and is what the A line on the UART carries.
    //
    //  INPUTS: the RAW engine outputs (tdoa_lag / tdoa_frac), not
    //  tdoa_lag_f / tdoa_frac_f.  The filtered pair is an IIR that only
    //  updates while a peak is valid, so at the instant the energy gate opens
    //  it still holds the PREVIOUS event -- reporting it would name whoever
    //  spoke last.  The raw per-frame ordering is noisy by ~0.4 sample but the
    //  microphones are 60 deg (6 samples) apart, so the ordering is safe.
    //
    //  STROBE: strobes[2], the same strobe tdoa_filter uses.  That is the
    //  cycle where valid_eff_r (written on strobes[1]) and the raw {lag,frac}
    //  bundle are both stable and belong to the same pass.
    //
    //  SECT_OFF_DEG / MIRROR_X / LED_REVERSE are passed through so the LED
    //  numbers follow a re-calibration automatically -- a hand-written table
    //  would silently keep reporting the old three LEDs.
    //==================================================================
    first_arrival #(
        .NMIC        (6),
        .FRAC_W      (FRAC_W),
        .NSEC        (NSEC),
        .SECT_OFF_DEG(SECT_OFF_DEG),
        .MIRROR_X    (MIRROR_X),
        .LED_REVERSE (LED_REVERSE),
        .GAP_MS      (EVT_GAP_MS),
        .CLK_HZ      (50_000_000)
    ) u_first (
        .clk        (clk_g),
        .rst_n      (rst_n_clk),
        .frame_done (strobes[2]),
        .lag        (tdoa_lag),
        .frac       (tdoa_frac),
        .valid      (valid_eff_r),
        .snd_now    (snd_now),
        .ev_stb     (ev_stb),
        .ev_mic     (ev_mic),
        .ev_tau     (ev_tau),
        .ev_l       (ev_l),
        .ev_c       (ev_c),
        .ev_r       (ev_r),
        .ev_sect    (),
        .ev_held    ()
    );

    //  BEARING SCALE -- this used to be out by a factor of two.
    //
    //  bearing is Q2.16: 65536 == pi == 180 deg, so 32768 == 90 deg.
    //  The packet field, mic_uart_framer.v's header and tools/doa_diag.py
    //  ALL agree on "Q1.14, 16384 == pi/2 == 90 deg" for bytes 16..17.
    //
    //  The old expression was `bearing[17:2]`, i.e. DIVIDE BY FOUR, which
    //  sends 8192 for a 90-degree source; doa_diag.py then decoded that as
    //  8192/16384*90 = 45 deg.  The bearing column was therefore reported
    //  at HALF its true value, which would have made a 2-degree jitter
    //  target look like 1 degree and handed back a wrong SECT_OFF
    //  calibration.  Dividing by TWO is what the documented scale needs.
    //
    //  A 18->16 bit arithmetic shift is exact here because |bearing| is
    //  bounded by 65536, so the shifted value is bounded by 32768, which
    //  fits a signed 16-bit word exactly.
    wire signed [17:0] bear_div2 = $signed(bear_lat) >>> 1;  // 16384 == 90 deg
    assign diag_bear  = bear_div2[15:0];
    assign diag_qual  = confidence[7:0];
    assign diag_sect  = sector;
    // doa_resid is Q(FRAC_W) = Q10 (1024 == one sample), the packet wants Q8
    // (256 == one sample), so this is a divide by four, NOT by 256.
    assign diag_resid = (doa_resid[15:2] > 14'd255) ? 8'd255 : doa_resid[9:2];
    assign diag_valid = tdoa_valid_f;

    //------------------------------------------------------------------
    // Peak level codes: how far each microphone's correlation peak sits
    // below the strongest one, in 6 dB steps (0 = strongest, 7 = 42+ dB).
    //
    //     A microphone that is dead, acoustically blocked, mis-mapped or
    //     unclocked shows up here at once -- and so does one that is merely
    //     20 dB down, which is the case the magnitude guard in section 5a
    //     acts on.  Read this column FIRST when the bearing is stubbornly
    //     wrong: one weak channel explains most large, repeatable errors.
    //------------------------------------------------------------------
    // NO SECOND MAX TREE AND NO COMPARATOR CHAIN.
    //
    // mag_top_r (section 5a) is already the registered maximum over the
    // VALID microphones, so it is reused here instead of building another
    // six-deep priority cone -- the same kind of cone that cost 9 ns of
    // setup slack when it sat in front of tdoa_filter.
    //
    // The code is the count of thresholds t_k = mag_top_r >>> k that the
    // microphone falls below, for k = 1..7.  Written as seven PARALLEL
    // comparisons feeding a 4-bit adder, because the obvious `for` loop of
    // chained compares and shifts is a seven-deep chain and measured badly
    // on this device.  top >>> k is pure wiring: no shifter hardware at all.
    //
    // tdoa_mag / tdoa_valid are still the current pass here (a new pass is
    // ~5.6 ms away) and doa_done arrives tens of clk later, so everything
    // this reads is stable when the UART framer latches the packet.
    function [2:0] lvl_code;
        input [ACC_W-1:0] m;
        input [ACC_W-1:0] peak;
        reg   [3:0]       s;
        begin
            s = {3'b0, (m < (peak >>> 7))}
              + {3'b0, (m < (peak >>> 6))}
              + {3'b0, (m < (peak >>> 5))}
              + {3'b0, (m < (peak >>> 4))}
              + {3'b0, (m < (peak >>> 3))}
              + {3'b0, (m < (peak >>> 2))}
              + {3'b0, (m < (peak >>> 1))};
            lvl_code = s[2:0];
        end
    endfunction

    // The reference level is mag_top_r -- the SAME registered maximum the
    // magnitude guard uses -- so the codes describe exactly the threshold
    // the guard applied.  (The previous version built its own combinational
    // maximum here; that tree plus the comparator chain was another 30 ns
    // cone of the kind that cost the guard its setup slack.)
    assign diag_lvl = { lvl_code(tdoa_mag[ACC_W*5 +: ACC_W], mag_top_r),
                        lvl_code(tdoa_mag[ACC_W*4 +: ACC_W], mag_top_r),
                        lvl_code(tdoa_mag[ACC_W*3 +: ACC_W], mag_top_r),
                        lvl_code(tdoa_mag[ACC_W*2 +: ACC_W], mag_top_r),
                        lvl_code(tdoa_mag[ACC_W*1 +: ACC_W], mag_top_r),
                        lvl_code(tdoa_mag[ACC_W*0 +: ACC_W], mag_top_r) };

    //==================================================================
    // 6.  LED ring + quality lamps.
    //
    //     `confidence` is a 0..255 QUALITY (see doa_solver.v): how many
    //     microphones fit one plane wave, and how well.  It measures
    //     microphone AGREEMENT, not loudness -- a quiet but consistent
    //     source scores high.  With the defaults (R_MAX = 1200,
    //     RF = 4096):
    //         >= CONF_MIN (128)    FOUR or more microphones agreed
    //     (higher thresholds, see the table at CONF_MIN, mean more
    //      microphones agreeing and/or a smaller residual)
    //
    //     BOTH the ring and leds[3] use it: sol_valid gates the sector vote
    //     (see the confirmation block) as well as the lamp, deliberately, so
    //     "what the ring shows" and "what leds[3] claims" can never disagree.
    //     CONF_MIN must stay in 0..255 (it is compared to the low byte).
    //==================================================================
    wire sol_valid = doa_done && (confidence[7:0] >= CONF_MIN);

    //------------------------------------------------------------------
    //  Published-bearing latch (see the bear_lat declaration).
    //
    //  Deliberately the SAME condition the ring is driven by, so "what the
    //  UART reports" and "what the ring shows" cannot disagree about whether
    //  a given frame was believable.  Held, not cleared, so a calibration
    //  reading taken while the source is held still stays put instead of
    //  decaying into the garbage frames.
    //------------------------------------------------------------------
    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk)
            bear_lat <= 18'sd0;
        else if (sol_valid && snd_now)
            bear_lat <= bearing;
    end

    //------------------------------------------------------------------
    //  RING DRIVE -- what actually gates it (rewritten 2026-11-01)
    //
    //  This block used to be titled "DIRECT RING DRIVE, no vote filter, no
    //  quality gate" and described the 2026-10-11 behaviour.  That has not
    //  been true since 2026-10-15/18, so the text is replaced rather than
    //  left to mislead:
    //
    //      frame accepted  <=>  sol_valid && snd_now
    //                           sol_valid = doa_done && confidence >= CONF_MIN
    //                           snd_now   = ref_energy > ANY_SND_E
    //      ring updates    <=>  accepted AND vote >= need AND dir_hold == 0
    //                           (confirmation block below, DIR_MIN_MS window)
    //
    //  i.e. there IS a quality gate and there IS a vote.  The history is why:
    //  driving every frame straight at the ring let the wrong sectors
    //  through, and on 2026-10-15 the capture showed Q = 00 / R = 00 (every
    //  microphone rejected) on about half the frames.
    //
    //  ANY_SND_E is the "there is sound" test -- the reference microphone's
    //  correlation-window energy.  A plain energy gate is the right
    //  primitive here: the question is "is anyone talking", not "is the
    //  estimate good".  Set it to 0 to disable the test (then noise lights
    //  the ring too).
    //
    //  dir_update pulses only when a NEW confirmed sector is committed, and
    //  the ring uses that pulse both to refresh and to reset its "no
    //  direction" timer.  A stationary talker therefore relies on DIR_MIN_MS
    //  expiring and the next confirmed frame re-committing -- not on a pulse
    //  every frame.
    //
    //  SECT_OFF still rotates the emitted sector for the mounting angle.
    //
    //  NOTE: snd_now / any_valid_g now live in section 5b-2, where the master
    //  energy gate is applied to the whole chain.  This block only USES them.
    //------------------------------------------------------------------
    //  sect_hold / dir_update are declared up by the framer instance (their
    //  users include its rpt_stb / rpt_sect ports) -- just assigned here.

    //  ---- 方位确认器 (2026-10-18) --------------------------------------
    //  sect_cand/sect_vote track "the sector the solver keeps insisting on".
    //  A commit needs `need` consecutive identical accepted solutions, where
    //  `need` is higher for a large angular hop (SECT_JUMP_SECT or more).
    //
    //  The counter is 4 bits and SATURATES at 15 -- wrapping would make a
    //  long steady state look like a fresh candidate every 16 frames.
    reg [3:0] sect_cand;
    reg [3:0] sect_vote;

    // 环上最短距离, 0..6  (a,b both < 12, so 4 bits are enough end to end --
    // declared this way on purpose: a 5-bit intermediate would trigger the
    // EX3791 truncation warning this project keeps having to chase)
    function [3:0] ring_dist;
        input [3:0] a;
        input [3:0] b;
        reg   [3:0] d;
        begin
            d = (a >= b) ? (a - b) : (b - a);
            ring_dist = (d > 4'd6) ? (4'd12 - d) : d;
        end
    endfunction

    //  vote_nx SATURATES.  Without the clamp, a candidate held for 15 frames
    //  wrapped to 0 and the ring stopped updating altogether -- a long steady
    //  talker would freeze the display (found by tb_top_io's throttle case).
    wire       cand_same = (sector == sect_cand);
    wire [3:0] cand_nx   = cand_same ? sect_cand : sector;
    wire [3:0] vote_nx   = cand_same ? ((sect_vote == 4'hF) ? 4'hF
                                                          : (sect_vote + 4'd1))
                                     : 4'd1;
    wire [3:0] hop_nx    = ring_dist(cand_nx, sect_hold);
    wire [3:0] need_nx   = (hop_nx >= SECT_JUMP_SECT) ? SECT_VOTES_FAR
                                                      : SECT_VOTES;

    //  UPDATE THROTTLE (added 2026-10-16)
    //
    //  A direction is pushed at the ring AT MOST once every DIR_MIN_MS, and
    //  the ring holds that direction for the whole window.  Without the
    //  throttle the ring followed the ~84 ms solve rate, so even a talker who
    //  was not moving saw the lit LEDs twitch from solve to solve.
    //
    //      sound arrives -> refresh now, light for DIR_MIN_MS
    //      inside the window -> new results IGNORED, display unchanged
    //      window expires -> the next CONFIRMED direction may be taken
    //
    //  dir_hold != 0 marks "inside the window".  It must stay strictly below
    //  RING_BLANK_MS: the ring's own blank timer is restarted by each
    //  new_dir, and an equal pair would blank at the very instant of the
    //  refresh (the ring would flicker dark once per window).
    localparam [LHW-1:0] DIR_MIN_CYC = DIR_MIN_MS * 50_000;   // 500 -> 25_000_000
    reg [LHW-1:0] dir_hold;

    //  NOTE 2026-10-16: the mounting rotation used to be applied right here,
    //  as `sect_hold + SECT_OFF` in whole sectors.  It moved into
    //  doa_solver (SECT_OFF_DEG) so it can be a continuous angle; this block
    //  now passes the solver's sector straight through.

    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            sect_hold  <= 4'd0;
            dir_update <= 1'b0;
            dir_hold   <= {LHW{1'b0}};
            sect_cand  <= 4'd0;
            sect_vote  <= 4'd0;
        end else begin
            dir_update <= 1'b0;

            if (dir_hold != {LHW{1'b0}})
                dir_hold <= dir_hold - 1'b1;

            //  sol_valid, not bare doa_done (changed 2026-10-15).
            //  doa_done pulses on every solved frame INCLUDING the ones the
            //  robust fit threw away: the board capture showed Q = 00 with
            //  R = 00 (ninl = 0, every microphone rejected) on roughly half
            //  the frames, and those frames' sector was still being pushed at
            //  the ring -- which is exactly the "ring flickers through random
            //  colours and never settles" report.  sol_valid already existed
            //  (it is what drives leds[3]); it just was not used here.
            if (sol_valid && snd_now) begin
                //  accumulate the confirmation vote (saturating, see vote_nx)
                sect_cand <= cand_nx;
                sect_vote <= vote_nx;

                //  commit only when the window is open AND the vote is in.
                //  Placed after the accumulator so it wins on the same edge.
                if ((dir_hold == {LHW{1'b0}}) && (vote_nx >= need_nx)) begin
                    sect_hold  <= cand_nx;
                    dir_update <= 1'b1;
                    dir_hold   <= DIR_MIN_CYC;
                    sect_vote  <= 4'd0;      // next change needs a fresh vote
                end
            end
        end
    end

    sk9822_ring #(
        .NLED        (12),
        .NSEC        (NSEC),
        .GLOBAL_BRI  (1),          // low, to keep supply noise off the mics
        .CORE_HZ     (50_000_000),
        .CLK_HZ      (2_500_000),  // slowed for margin on the 1.5 V bus
        .HOLD_CYCLES (1_000_000),  // ~20 ms quiet gap between refreshes
        .BLANK_CYCLES(RING_BLANK_MS * 50_000),
        .LED_REVERSE (LED_REVERSE)
    ) u_ring (
        .clk         (clk_g),
        .rst_n       (rst_n_clk),
        .new_dir     (dir_update),
        //  sect_q, not sect_hold (2026-11-01).  sect_hold is the 30 deg bin
        //  the solver voted on; sect_q is that bin re-centred on the axis of
        //  the microphone it belongs to.  The ring, the LED numbers on the
        //  UART line and M= all read sect_q, so a 3-LED group always means
        //  "microphone k", which is the only mapping the printed table claims.
        .sector      (sect_q),
        .active_span (RING_SPAN[3:0]),
        .led_clk     (mic_led_clk),
        .led_dat     (mic_led_dat),
        .lit         (ring_lit)      // the A line's G= field
    );

    //  What the ring is actually showing (throttled by DIR_MIN_MS), in the
    //  display domain -- sect_q, so the "F" report quotes the same value the
    //  lamps were built from instead of the raw 30 deg vote.
    assign rpt_sect = sect_q;

    //==================================================================
    // 7.  PMOD LEDs
    //
    //     These four LEDs carry the slow, human-readable status; the ring
    //     carries the actual answer.  The heartbeat is free-running so you
    //     can tell the design is alive even when no audio is arriving.
    //==================================================================
    reg [24:0] heartbeat;
    reg        drop_sticky;

    // Debug latch: keeps the raw bearing, the vector magnitude and the
    // captured energy observable without adding a pin.  They drive no logic,
    // so their only purpose is to survive synthesis and remain selectable
    // nets for the on-chip analyser / netlist viewer.
    //
    // Attribute note -- measured, not assumed.  GowinSynthesis silently
    // IGNORES `syn_keep` on a register (and on a wire): these four were
    // deleted and `grep dbg_ .../micarray.vg` returned 0.  A minimal
    // one-module experiment on this exact toolchain (V1.9.12.03) compared
    // every candidate attribute:
    //     syn_keep      -> ignored      syn_probe    -> ignored
    //     syn_noprune   -> ignored      no attribute -> deleted
    //     syn_preserve  -> KEPT as real DFFRE primitives
    //     syn_dont_touch-> KEPT as real DFFRE primitives
    // so `syn_preserve` is used below.  Cost: 18+36+8+24 = 86 extra
    // registers.  Re-check any time with
    //     grep -c "dbg_bearing" impl/gwsynthesis/micarray.vg     # expect != 0
    reg signed [17:0]      dbg_bearing /* synthesis syn_preserve=1 */;
    reg signed [ACC_W-1:0] dbg_energy  /* synthesis syn_preserve=1 */;
    reg [7:0]              dbg_bytes   /* synthesis syn_preserve=1 */;
    reg [23:0]             dbg_conf    /* synthesis syn_preserve=1 */;

    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            heartbeat   <= 25'd0;
            drop_sticky <= 1'b0;
            dbg_bearing <= 18'sd0;
            dbg_energy  <= {ACC_W{1'b0}};
            dbg_bytes   <= 8'd0;
            dbg_conf    <= 24'd0;
        end else begin
            heartbeat <= heartbeat + 1'b1;
            if (dropped != 16'd0)
                drop_sticky <= 1'b1;
            if (doa_done) begin
                dbg_bearing <= bearing;
                dbg_conf    <= confidence;
                dbg_energy  <= ref_energy;
            end
            dbg_bytes <= bytes_sent;
        end
    end

    // (The microphone-activity detector was DELETED: the diagnostic UART
    //  packet's level codes / valid mask answer "are the microphones alive"
    //  in far more detail, and leds[2]/leds[3] are more useful as the
    //  ladder below.  To bring the old check back, drive a spare LED from a
    //  comparator on ch6.)

    //------------------------------------------------------------------
    //  LED OUTPUT STAGE
    //
    //  LED_SELFTEST = 1 drives a fixed pattern that depends on nothing but
    //  the 50 MHz clock, so it proves the four LEDs and their pin bindings
    //  are good on their own:
    //      leds[0] = 1             constant on
    //      leds[1] = 1 Hz blink    0.5 s on / 0.5 s off
    //      leds[2] = 0             constant off
    //      leds[3] = 1             constant on
    //  LED_SELFTEST = 0 (the default) drives the live ladder.  READ leds[2]
    //  FIRST: it selects which table leds[1]/leds[3] belong to.
    //
    //  --- leds[2] LIT -> a correlation peak exists AND the reference
    //                     microphone heard something (any_valid_g), so read
    //                     HOW GOOD ---
    //      leds[1] = the centre reference is delivering data
    //      leds[2] = the engine found at least one believable peak
    //      leds[3] = the solution reached CONF_MIN (worth believing)
    //
    //  --- leds[2] DARK -> no usable peak, so read WHAT IS MISSING ---
    //      leds[1] = reference status, encoded in the BLINK RATE
    //                (slower = healthier, so a worsening board blinks faster):
    //                  solid   the centre mic (the MAP6 phase) is moving
    //                  ~1.5 Hz it is NOT, but a RING mic (lane 0..2) is --
    //                          the array is alive, only the reference path
    //                          is broken
    //                  ~3 Hz   nothing anywhere is moving
    //      leds[2] = dark by definition (no peak)
    //      leds[3] = the OTHER lane-3 phase is delivering data
    //
    //  THE ONE-LINE FIX, and how the board points at it:
    //      leds[0] lit + leds[1] blinking + leds[3] LIT
    //        -> the centre microphone's L/R strap puts it in the lane-3 phase
    //           that MAP6 did NOT select.  Put MAP6 on the other of 6 / 7.
    //           (Found this way on 2026-10-15: with MAP6 = 6 the board
    //            reported led0 lit / led1 blinking / led2+led3 dark.  Without
    //            u_mon7 that is indistinguishable from a dead array, because
    //            either way no correlation ever sees anything.)
    //      leds[0] lit + leds[1] blinking at ~3 Hz + leds[3] dark
    //        -> neither lane-3 phase answers and no ring mic moves either:
    //           supply, CHIPEN, the flying leads or the 3.3 V level problem.
    //           Hardware, not parameters.
    //
    //  The centre microphone's ABSOLUTE level is on the UART status line's
    //  E= field (7 level codes, one per channel), which is why there is no
    //  lamp for it any more.
    //------------------------------------------------------------------
    localparam integer BLINK_HALF = 25_000_000;     // 0.5 s at 50 MHz
    localparam [24:0]  BLINK_LAST = BLINK_HALF - 1; // 25 bits: 0..33.5 M

    reg [24:0] blink_cnt;
    reg        blink;

    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            blink_cnt <= 25'd0;
            blink     <= 1'b0;
        end else if (blink_cnt == BLINK_LAST) begin
            blink_cnt <= 25'd0;
            blink     <= ~blink;
        end else begin
            blink_cnt <= blink_cnt + 1'b1;
        end
    end

    // The two lamps are a LADDER, not two versions of one test.  A
    // one-cycle pulse (20 ns) would be invisible, so each event is stretched
    // to LED_HOLD = 0.5 s -- which also makes them steady lamps while
    // somebody is talking.
    //
    //   leds[2] = "the chain is alive": at least one solution was produced
    //             (doa_done).  Dark here while you talk means the
    //             correlation/solver chain is producing NOTHING -- look at
    //             microphone data, clocks and wiring long before quality.
    //   leds[3] = "the solution is usable": quality reached CONF_MIN, i.e.
    //             roughly three microphones agreeing on one plane wave (see
    //             the table at CONF_MIN).  Dark with leds[2] lit means
    //             solutions exist but the microphones disagree: SNR,
    //             reverberation, or one bad channel.
    //
    // With the ring, that is the whole diagnosis in three lamps:
    // ring lit = sound passed ANY_SND_E, leds[2] = a bearing was computed,
    // leds[3] = it is worth believing.
    // LHW (hold-counter width) is declared in the parameter section above,
    // next to DIR_MIN_MS, because the ring throttle needs it too.
    localparam [LHW-1:0] LED_HOLD = 500 * 50_000;      // 0.5 s at 50 MHz

    // NOTE (2026-10-15): the sol_hold / doa_seen pair that used to live here
    // was DELETED.  It stretched doa_done into a 0.5 s "a solution was
    // produced" flag, but nothing has read it since leds[2] was changed to
    // anyv_seen, so both the wire and the 25-bit countdown were dead logic
    // (25 registers plus a decrementer) and the comment claiming it was
    // "kept" was describing a signal with no readers.  The UART Q/T line
    // carries the same information.  To bring the lamp back, restore
    //     reg [LHW-1:0] sol_hold;  (reload on doa_done, decrement otherwise)
    //     wire doa_seen = (sol_hold != {LHW{1'b0}});
    // and drive leds[2] from it instead of anyv_seen.
    reg [LHW-1:0] qual_hold;

    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk) begin
            qual_hold <= {LHW{1'b0}};
        end else begin
            if (sol_valid)
                qual_hold <= LED_HOLD;
            else if (qual_hold != {LHW{1'b0}})
                qual_hold <= qual_hold - 1'b1;
        end
    end

    wire qual_seen = (qual_hold != {LHW{1'b0}});

    //------------------------------------------------------------------
    //  anyv_hold: "the engine found at least one believable peak",
    //  stretched like the other events (2026-10-14).
    //
    //  THIS IS THE RUNG THAT WAS MISSING.  leds[2] used to show doa_seen,
    //  which the solver only sets after it has RUN -- and it refuses to run
    //  when any_valid is 0.  So a dead reference channel (every correlation
    //  exactly 0, best = 0, any_valid = 0) looked exactly like "no data at
    //  all" on the board: leds[2] dark either way.  Driving the lamp from
    //  any_valid instead splits the two, and leds[1] splits the next one:
    //      leds[1] blinks + leds[2] dark -> no data reaches the FPGA
    //      leds[1] solid  + leds[2] dark -> data, but no peak: level too
    //                                       low, dead reference, wrong MAP
    //      leds[2] lit    + leds[3] dark -> peaks, microphones disagree
    //      both lit                      -> tracking
    //
    //  CHANGED 2026-10-15: driven by any_valid_g (the energy-gated version),
    //  not by the raw any_valid.  The raw signal is "best > 0", which room
    //  noise satisfies on every frame, so leds[2] was permanently lit in a
    //  silent room.  It now means "a peak was found AND the reference
    //  microphone actually heard something".
    //------------------------------------------------------------------
    localparam [LHW-1:0] ANYV_HOLD = 500 * 50_000;      // 0.5 s at 50 MHz

    reg [LHW-1:0] anyv_hold;

    always @(posedge clk_g or negedge rst_n_clk) begin
        if (!rst_n_clk)
            anyv_hold <= {LHW{1'b0}};
        else if (any_valid_g)
            anyv_hold <= ANYV_HOLD;
        else if (anyv_hold != {LHW{1'b0}})
            anyv_hold <= anyv_hold - 1'b1;
    end

    wire anyv_seen = (anyv_hold != {LHW{1'b0}});

    // led0 is the FIRST rung of the ladder: I2S framing acquired.  It proves
    // BCK/WS are really leaving the FPGA and the array answered with a
    // well-formed frame.  Without this lamp the ladder starts one rung too
    // high -- and a dead clock output then looks exactly like dead
    // microphones (see the history note at the I2S master section: that is
    // precisely the failure the first revision had).
    //
    // This assign was MISSING until 2026-10-15.  leds[] is a 4-bit port and
    // only [1], [2], [3] were driven, so led0 was left undriven by synthesis
    // while the header above still documented it as mic_locked.
    assign leds[0] = LED_SELFTEST ? 1'b1 : mic_locked;

    // led1 is the REFERENCE rung, and with no peaks its BLINK RATE says how
    // far the damage goes -- slower is healthier, so a board that is
    // degrading blinks faster (see the table at the LED OUTPUT STAGE):
    //      solid   the centre mic (the MAP6 phase) is moving
    //      ~1.5 Hz it is not, but a RING mic is: the array is alive and only
    //              the reference path is broken
    //      ~3 Hz   nothing anywhere is moving: supply / CHIPEN / wiring
    wire blink_slow = heartbeat[24];       // ~1.5 Hz at 50 MHz
    wire blink_fast = heartbeat[23];       // ~3 Hz
    assign leds[1] = LED_SELFTEST ? blink
                   : (ref_alive  ? 1'b1
                                 : (ring_alive ? blink_slow : blink_fast));

    assign leds[2] = LED_SELFTEST ? 1'b0 : anyv_seen;      // off / peaks found

    // led3 carries two meanings, and leds[2] says which one is active:
    //   peaks exist (led2 lit) -> quality >= CONF_MIN
    //   no peaks   (led2 dark) -> the OTHER lane-3 phase is delivering data,
    //                             i.e. the centre microphone is strapped to
    //                             the phase MAP6 did not select.  THAT is the
    //                             one-line fix, and this lamp is what makes it
    //                             readable instead of a guess between
    //                             "wrong MAP6" and "dead array".
    //
    // The old no-peak meaning here was (ref_alive && ref_loud) -- "the
    // reference is loud enough for the engine's x>>8".  That could never
    // separate the two cases above (both read 0), and the reference level is
    // on the UART status line's E= field anyway, so it was replaced rather
    // than kept and left unread.
    assign leds[3] = LED_SELFTEST ? 1'b1
                   : (anyv_seen ? qual_seen : slot_other_alive);

endmodule


//-----------------------------------------------------------------------------
// audio_ch_mon -- activity + absolute level monitor for ONE raw microphone channel.
//
//   Instantiated seven times by top.v section 3b (ch0..ch6).  That section
//   explains why the CENTRE REFERENCE has to be visible: a silent reference
//   makes every correlation exactly zero, the engine's validity rule
//   (best > 0) then reports no peak at all, and the solver never runs.
//
//   act : held for 0.5 s after the raw sample moved by more than ACT_TH
//         between two audio frames.  A stuck line, an unpowered microphone
//         or a missing bit clock reads 0 forever.
//   lvl : log2 of the running mean of |x|, one 6 dB step per count.  The
//         mean has a ~256-frame (5 ms) time constant and a 256 LSB
//         resolution floor, so a truly flat channel reads 0.
//
//   The log2 is a 25-cycle SERIALISER, not a 25-deep priority chain: it
//   runs once per audio frame (1024 clock cycles), so it costs 2.4% of the
//   time and none of the 50 MHz timing margin.
//-----------------------------------------------------------------------------
module audio_ch_mon #(
    parameter integer DATA_W = 24,
    parameter integer ACT_TH = 512
) (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     stb,     // one pulse per audio frame
    input  wire signed [DATA_W-1:0] x,
    output wire                     act,
    output reg  [4:0]               lvl
);
    localparam integer   ACT_HOLD = 25_000_000;          // 0.5 s at 50 MHz
    localparam [4:0]     LST      = DATA_W;              // 5'd24 for 24-bit

    reg signed [DATA_W-1:0] xp;                          // previous frame

    // |x| as an unsigned magnitude.  -2^23 negates to +2^23, which still
    // fits the 24-bit pattern as 0x800000, so no extra bit is needed.
    wire [DATA_W-1:0] mag = x[DATA_W-1] ? (~x + 1'b1) : x;

    wire moved = ((x - xp) > ACT_TH) || ((xp - x) > ACT_TH);

    reg [24:0]      hold;
    reg [DATA_W:0]  mean;      // running mean of mag, same scale
    reg [4:0]       ei;
    reg             run;

    assign act = (hold != 25'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xp   <= {DATA_W{1'b0}};
            hold <= 25'd0;
            mean <= {(DATA_W+1){1'b0}};
            ei   <= 5'd0;
            run  <= 1'b0;
            lvl  <= 5'd0;
        end else if (stb) begin
            xp <= x;
            if (moved)
                hold <= ACT_HOLD;
            mean <= mean - (mean >> 8) + (mag >> 8);
            ei   <= 5'd0;
            run  <= 1'b1;
        end else if (run) begin
            // walk 0..DATA_W: the LAST match is the highest set bit
            if (ei == 5'd0)
                lvl <= 5'd0;
            else if (mean[ei])
                lvl <= ei;

            if (ei == LST)
                run <= 1'b0;
            else
                ei <= ei + 1'b1;
        end else if (hold != 25'd0) begin
            hold <= hold - 1'b1;
        end
    end
endmodule
