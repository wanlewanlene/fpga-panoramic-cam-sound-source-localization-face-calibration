
create_clock -name clk -period 20.0 -waveform {0 10} [get_ports {clk}]

# PLL1 (Gowin_PLL_m0)
create_generated_clock -name cam_xclk -source [get_ports {clk}] -master_clock clk -divide_by 50  -multiply_by 24 [get_pins {Gowin_PLL_m0/PLLA_inst/CLKOUT0}]
create_generated_clock -name caux_clk -source [get_ports {clk}] -master_clock clk -divide_by 127 -multiply_by 24 [get_pins {Gowin_PLL_m0/PLLA_inst/CLKOUT1}]
create_generated_clock -name mem_clk  -source [get_ports {clk}] -master_clock clk -divide_by 3   -multiply_by 24 [get_pins {Gowin_PLL_m0/PLLA_inst/CLKOUT2}]

# PLL2 (u_tmds_pll)
create_generated_clock -name clk_tmds_5x -source [get_ports {clk}] -master_clock clk -divide_by 40  -multiply_by 297 [get_pins {u_tmds_pll/PLLA_inst/CLKOUT0}]
create_generated_clock -name clk_74_25   -source [get_ports {clk}] -master_clock clk -divide_by 200 -multiply_by 297 [get_pins {u_tmds_pll/PLLA_inst/CLKOUT1}]

# DDR PLL
create_generated_clock -name clk_x1 -source [get_pins {Gowin_PLL_m0/PLLA_inst/CLKOUT2}] -master_clock mem_clk -divide_by 4 -multiply_by 1 [get_pins {u_ddr3/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]

create_clock -name cmos_pclk -period 41.67 -waveform {0 20.835} [get_ports {cmos_pclk}]
create_clock -name cmos2_pclk -period 41.67 -waveform {0 20.835} [get_ports {cmos2_pclk}]
create_clock -name cmos3_pclk -period 41.67 -waveform {0 20.835} [get_ports {cmos3_pclk}]

create_generated_clock -name cmos_16bit_clk_gen -source [get_ports {cmos_pclk}] -divide_by 2 [get_nets {cmos_16bit_clk}]
create_generated_clock -name cmos2_16bit_clk_gen -source [get_ports {cmos2_pclk}] -divide_by 2 [get_nets {cmos2_16bit_clk}]
create_generated_clock -name cmos3_16bit_clk_gen -source [get_ports {cmos3_pclk}] -divide_by 2 [get_nets {cmos3_16bit_clk}]


//=============================================================================
//  [合并工程] 音频子系统时序约束  ——  来源 audio_i2s/eda_proj/src/audio.sdc
//
//  FPGA 是 I2S 主机: MIC_CK = clk/16 = 3.125 MHz (周期 320 ns),
//  MIC_WS = BCK/64 = 48.83 kHz。BCK 由引脚输出，因此按输出端口上的
//  独立时钟约束；麦克风在 BCK 下降沿更新数据，用 input_delay 描述返回路径。
//
//  这里刻意用 create_clock 而不是 create_generated_clock: 原工程实测过，
//  若让工具认为 mic_bck 与 clk 同源，它会按边沿对齐做常规 setup/hold 检查，
//  报出 26 条落在 mic_bck<->clk 跨域路径上的假 hold 违例 (最差 -1.012 ns)。
//  实际跨域靠「翻转同步器 + 整组样本保持一整帧不变」完成，见 audio_top.v。
//=============================================================================
create_clock -name mic_bck -period 320.0 -waveform {160 320} [get_ports {mic_array_bck}]

set_input_delay -clock mic_bck -max 30.0 [get_ports {mic_array_d[*]}]
set_input_delay -clock mic_bck -min  5.0 [get_ports {mic_array_d[*]}]

// bck <-> clk 的跨域: 用 max_delay 兜住物理发射-捕获延迟, 不假装同步关系
// (Gowin 解析器不接受 -datapath_only, 因此用普通形式)
set_max_delay -from [get_clocks {mic_bck}] -to [get_clocks {clk}] 40.0
set_max_delay -from [get_clocks {clk}] -to [get_clocks {mic_bck}] 40.0

set_clock_groups -asynchronous 
    -group [get_clocks {clk cam_xclk caux_clk mem_clk clk_tmds_5x clk_74_25 clk_x1}] 
    -group [get_clocks {cmos_pclk}] 
    -group [get_clocks {cmos2_pclk}] 
    -group [get_clocks {cmos3_pclk}] 
    -group [get_clocks {mic_bck}]
# HSPI 诊断口：hspi_clk = clk/4 = 12.5 MHz，输出/输入均按 clk(20ns) 约束
set_output_delay -clock clk -max 2.0 [get_ports {hspi_htclk hspi_htreq hspi_htvld hspi_hd[*]}]
set_input_delay  -clock clk -max 2.0 [get_ports {hspi_htrdy}]

# ------------------------------------------------------------------------------
# 人脸检测结果跨域：clk_74_25（视频域）→ clk
#
#   跨的是什么：face_det_stream 在视频域输出 face_cx/cy/w/h/conf + face_tgl，
#   pan_ctrl 在 clk 域接收。做法是"数据稳定整帧（66.7 ms）+ toggle 两级同步"，
#   即握手式 CDC：数据在 toggle 翻转前就已稳定，接收端等同步后的 toggle 边沿
#   才采样，因此这条路径没有"单周期时序"要求。
#
#   为什么用 set_false_path 而不是 set_max_delay：
#     · -datapath_only（唯一能去掉时钟延迟项的写法）在 Gowin V1.9.11.03
#       **不支持**（TA2000 语法错，构建直接失败）；
#     · 不用 -datapath_only 的 set_max_delay 对"同一 set_clock_groups 组内
#       的派生时钟"实测无效（TNS 不随参数变化）。
#
#   适用范围：本设计里 clk_74_25 → clk 只有这一处跨域（face_* → pan_ctrl），
#   所以不会掩盖其他真实路径。
# ------------------------------------------------------------------------------
set_false_path -from [get_clocks {clk_74_25}] -to [get_clocks {clk}]
