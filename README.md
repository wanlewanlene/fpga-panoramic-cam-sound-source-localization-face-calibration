# FPGA 全景相机 · 声源定位 · 人脸标定

FPGA Panoramic Camera · Sound-Source Localization · Face Calibration

基于高云 Gowin **GW5AT-60B**（Sipeed Tang MEGA 60K）的 360° 全景拼接 + 7 麦克风声源定位（DOA）+ 人脸检测的一体化 FPGA 工程。三路 OV5640 鱼眼相机拼接成 360° 全景，7 麦圆阵解算声源方位，画面自动转向说话者所在方向。

> 本仓库源码为 **v1.1.23** 版本（UserCode `0x2B1C`）。

---

## 功能特性

- **360° 全景视频**：3×OV5640 鱼眼 → 8→16bit → 1280 深行缓冲（3 路，组合读，走分布式 RAM）→ 重叠区 alpha 融合 → 鱼眼畸变校正 LUT → DDR3 帧缓存 → HDMI 720P60
- **声源定位（DOA）**：7 麦圆阵（6 环 + 1 中心）→ I2S 主模式采集 → 4 子带 PHAT 互相关 → 抗差最小二乘 → 方位角 + 12 灯环指示
- **人脸检测**：画面中人脸检测与框选
- **自动转向**：声源方位驱动取景窗在 360° 全景上平移，无音源时自动停驻
- **串口诊断**：视频诊断 + 音频诊断两路 UART 字节级互斥合流。注意实际能读的是
  **G11**（DONE 专用脚释放，115200 8N1，接 USB-TTL/CH340）；`uart_pc_tx` 虽然接到
  板载桥片的 U15，但实测**该脚到不了 PC**（多波特率均为 0 字节），详见
  `docs/MIC_ARRAY_I2S_DESIGN.md` §7.2

## 硬件平台

| 项 | 规格 |
|---|---|
| FPGA | 高云 GW5AT-LV60PG484（Tang MEGA 60K） |
| 相机 | 3× OV5640（鱼眼镜头，单路约 150° FOV） |
| 麦克风 | 7× MSM261S4030H0（6 环形 + 1 中心参考） |
| 灯环 | 12× SK9822（指示声源方位扇区） |
| 输出 | HDMI 720P60（TMDS） |

## 目录结构

```
.
├── cam_mic.gprj                 工程文件
├── build.tcl                    一键构建（run all）
├── build_syn.tcl                只跑综合（改代码后快速查错）
├── settings/
│   └── project_process_config.json    ★ Enable_DSRM=true，见下"坑 1"
├── src/
│   ├── top.v                    唯一顶层：视频链路 + 音频例化 + UART 合流
│   ├── cam_mic_merge.cst        合并后的引脚约束
│   ├── cam_mic_merge.sdc        合并后的时序约束
│   ├── audio/                   声源定位 RTL（audio_top / tdoa_engine /
│   │                             doa_solver / sk9822_ring / mic_capture ...）
│   ├── face_det_stream.v        人脸检测
│   ├── pan_ctrl.v               自动转向控制
│   └── ...                      视频链路 RTL 与 IP 网表（ddr3 / dvi_tx / fifo / pll）
└── impl/                        构建产物（不入库，见 .gitignore）
```

## 构建

需要 Gowin IDE（V1.9.11 或 V1.9.12）命令行工具 `gw_sh.exe`：

```bash
gw_sh.exe build.tcl        # 完整流程，约 9.5 分钟
gw_sh.exe build_syn.tcl    # 只综合，约 40 秒
```

产物：`impl/pnr/cam_mic.fs`（位流，本版本 UserCode `0x2B1C`）。烧录：

```bash
programmer_cli --device GW5AT-60B --operation_index 4 \
               --cable "Gowin USB Cable(FT2CH)" \
               --fsFile impl/pnr/cam_mic.fs
```

位流开了 SecurityBit，烧完报 `Verify Failed` 属正常；以「回读 UserCode == 位流头
`//CheckSum`」且「Status bit13 Done=1」为成功判据。`--operation_index 4` 是下到 SRAM，
掉电即失效。

### 三个必须知道的坑

1. **`Enable_DSRM` 必须是 `true`**（`settings/project_process_config.json`）。
   三个行缓冲是组合读，无法映射到 BSRAM，只能走分布式 RAM；不打开会报
   `ERROR (RP0007) : There is no SSRAM resource in current device`。
2. **必须显式指定顶层**：`build.tcl` 里的 `set_option -top_module top` 不能删，
   否则综合器会随机挑一个未例化的模块当顶层，编出 10 KB 空壳却打印"综合成功"。
   判断标准：真实网表 `impl/gwsynthesis/cam_mic.vg` 约 7.7 MB，空壳只有 10~46 KB。
3. **`asynchronous` 时钟组要包含音频**：`cam_mic_merge.sdc` 里 `set_clock_groups`
   的最后一组是 `mic_bck`，删掉会报一堆假的 hold 违例。

## 资源占用（GW5AT-LV60PG484）

**含人脸检测的完整版本**（v1.1.23 同代码族实测，V1.9.11 编译）：

| 资源 | 占用 | 利用率 |
|---|---|---|
| Logic | ≈39.2 K / 59.9 K | 66 % |
| CLS | ≈23.8 K / 29.9 K | **80 %** |
| BSRAM | 54 / 118 | 46 % |
| DSP | 26.5 / 118 | 22 % |
| I/O | 143 / 297 | 49 % |

**CLS 80% 是本工程最紧的一项**，加功能前务必先看编译报告——实测再加 24 个末级
寄存器就会 `PR0004` 布线失败。作为参照：关掉人脸检测（`FACE_EN=0`）可省下
**约 3% CLS + 4.5 DSP**（CLS 降到 77%），那 3% 就是后续做连续跟踪的余量。

> 注：工程内 `mix/cam_mic_merge/README.md` 里那张 53% / 68% 的表是**视频+音频整合
> 阶段**（人脸未并入）的数据，不要当作完整版指标。具体数值以你本机 `build.tcl`
> 报告为准，EDA 小版本间会有 ±2% 浮动。

## 上板自检顺序

按顺序检查，第一个不亮/不对的就是故障点：

| 步 | 观察点 | 正常表现 |
|---|---|---|
| 1 | `state_led[0..4]` | 视频诊断计数器在动 |
| 2 | `mic_leds[0]`（P14） | 常亮（BCK/WS 有输出） |
| 3 | `mic_leds[1]`（P15） | 常亮（参考麦有数据） |
| 4 | `mic_leds[2]`（U17） | 有声音时亮 |
| 5 | `mic_leds[3]`（N15） | 解算质量达标时亮 |
| 6 | 灯环（12 颗） | 指向声源扇区 |
| 7 | `diag_uart_tx`（**G11**，DONE 脚释放）@115200 8N1 | 同时出现音频 `Q=.. R=..` 行和视频诊断行（U15 实测无通路，别接那里） |
| 8 | HDMI | 720P60 全景，按键左右旋转 |

## 已知限制

1. `vga_timing` 有 2 条约 −5.7 ns 的时序违例未处理（`clk_74_25` 频率声明与 PLL 实际
   defparam 不一致，不影响出图，但使时序报告失真）。
2. `cmos2_16bit_clk` 降级走 LW 长线（PRIMARY 时钟网络已满），cam2 画面若出现横向
   条纹或抖动优先怀疑这里。
3. UART 冲突时视频优先、音频让路，被让路一方丢整字节（不会乱码）。
4. 麦克风阵列需单独 5V 供电，飞线要短、GND 伴线。

## 版本

| 版本 | 说明 |
|---|---|
| **v1.1.23** | 本仓库源码。修画面倾斜（VSYNC 帧锁存）+ 3 秒转向 + 无音源禁用自动 |

## 许可证

MIT License（见 `LICENSE`，Copyright © 2026 玩了完了呢）。仅作技术研究与学习参考。
