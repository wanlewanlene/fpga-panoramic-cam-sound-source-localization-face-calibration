
module top #(
    parameter USE_TPG = "false",
    parameter DDR_BYPASS = "false"
)(
	input                  clk,
	input                  rst_n,      
    // --- 一号摄像头 ---
	inout                  cmos_scl,       //cmos i2c clock
	inout                  cmos_sda,       //cmos i2c data
	input                  cmos_vsync,     //cmos vsync
	input                  cmos_href,      //cmos hsync refrence,data valid
	input                  cmos_pclk,      //cmos pxiel clock
	input  [7:0]           cmos_db,        //cmos data
                                           // end
    // --- 二号摄像头 ---
    inout                  cmos2_scl,      
    inout                  cmos2_sda,
    input                  cmos2_vsync,
    input                  cmos2_href,
    input                  cmos2_pclk,
    input  [7:0]           cmos2_db,       // end
    // --- 三号摄像头 ---
    inout                  cmos3_scl,
    inout                  cmos3_sda,
    input                  cmos3_vsync,
    input                  cmos3_href,
    input                  cmos3_pclk,
    input  [7:0]           cmos3_db,       // end

    // --- 按键输入 ---
    input                  key_left,       // 连接到 AB13
    input                  key_right,      // 连接到 Y12
	output                 cmos_rst_n,     //cmos reset 
	output                 cmos_pwdn,      //cmos power down
	
	output [4:0]           state_led,

    // --- HSPI 诊断口（观测计数器 → CH569 → USB-CDC 到 PC）---
	output                 hspi_htclk,     // J20
	output                 hspi_htreq,     // J21
	input                  hspi_htrdy,     // L19
	output                 hspi_htvld,     // G18
	inout  [15:0]          hspi_hd,        // G17 H22 L14 K22 K21 K18 L21 M21 M22 N22 L18 M18 N19 N18 K16 L16
	output                 diag_uart_tx,   // G11(DONE 脚释放) 115200 文本诊断行，供 CH340/USB-TTL 直读
	output                 uart_pc_tx,     // U15(板载 USB 桥片) 视频诊断 + 麦克风诊断合流后的一路串口

	// --- 麦克风阵列（7 麦 I2S 飞线，3.3V；模块实现在 src/audio/audio_top.v）---
	output mic_array_bck,   // A13  MIC_CK  3.125 MHz (FPGA 作 I2S 主模式输出)
	output mic_array_ws,    // A14  MIC_WS  48.83 kHz (FPGA 作 I2S 主模式输出)
	input  [3:0] mic_array_d,     // B13/B15/B16/C13 = MIC_D0/MIC_D1/MIC_D2/MIC_D3
	output mic_led_clk,     // C14  SK9822 灯环时钟 (2.5 MHz)
	output mic_led_dat,     // C15  SK9822 灯环数据
	output [3:0] mic_leds,        // P14/P15/U17/N15  麦克风状态灯（H3 排针）

	output [15:0]          ddr_addr,       //ROW_WIDTH=16
	output [2:0]           ddr_bank,       //BANK_WIDTH=3
	output                 ddr_cs,
	output                 ddr_ras,
	output                 ddr_cas,
	output                 ddr_we,
	output                 ddr_ck,
	output                 ddr_ck_n,
	output                 ddr_cke,
	output                 ddr_odt,
	output                 ddr_reset_n,
	output [1:0]           ddr_dm,         //DM_WIDTH=4
	inout  [15:0]          ddr_dq,         //DQ_WIDTH=32
	inout  [1:0]           ddr_dqs,        //DQS_WIDTH=4
	inout  [1:0]           ddr_dqs_n,      //DQS_WIDTH=4
  
    output                 tmds_clk_n_0,
    output                 tmds_clk_p_0,
    output [2:0]           tmds_d_n_0, //{r,g,b}
    output [2:0]           tmds_d_p_0,
    output                 hpd_en
);

//=============================================================================
//  音频子系统 / UART 合流的顶层连线
//    uart_vid_tx2 : cam_diag 的第二路文本输出（原直连 U15）
//    uart_aud_tx  : audio_top 的 DOA 文本输出
//    uart_pc_tx   : 两路经 uart_merge 互斥合流后真正驱动 U15 的那根线
//    uart_conflict: 两路曾同时请求线路的粘滞标志（调试观察用）
//=============================================================================
wire uart_vid_tx2;
wire uart_aud_tx;
wire uart_conflict;

//memory interface
    wire                   memory_clk         ;
    wire                   dma_clk         	  ;
    wire                   DDR_pll_lock       ;
    wire                   cmd_ready          ;
    wire[2:0]              cmd                ;
    wire                   cmd_en             ;
    //wire[5:0]              app_burst_number   ;
    wire[ADDR_WIDTH-1:0]   addr               ;
    wire                   wr_data_rdy        ;
    wire                   wr_data_en         ;
    wire                   wr_data_end        ;
    wire[DATA_WIDTH-1:0]   wr_data            ;   
    wire[DATA_WIDTH/8-1:0] wr_data_mask       ;   
    wire                   rd_data_valid      ;  
    wire                   rd_data_end        ;//unused 
    wire[DATA_WIDTH-1:0]   rd_data            ;   
    wire                   init_calib_complete;
    wire                   err;
    wire                   TMDS_DDR_pll_lock  ;

    //According to IP parameters to choose
    `define	    WR_VIDEO_WIDTH_32
    `define	DEF_WR_VIDEO_WIDTH 32

    `define	    RD_VIDEO_WIDTH_32
    `define	DEF_RD_VIDEO_WIDTH 32

    `define	USE_THREE_FRAME_BUFFER

    `define	DEF_ADDR_WIDTH 29 
    `define	DEF_SRAM_DATA_WIDTH 128
    
    //=========================================================
    //SRAM parameters
    parameter ADDR_WIDTH          = `DEF_ADDR_WIDTH;        
    parameter DATA_WIDTH          = `DEF_SRAM_DATA_WIDTH;   
    parameter WR_VIDEO_WIDTH      = `DEF_WR_VIDEO_WIDTH;  
    parameter RD_VIDEO_WIDTH      = `DEF_RD_VIDEO_WIDTH;  
    wire                            video_clk;  //video pixel clock
    //-------------------
    //syn_code
    wire                      syn_off0_re;      // ofifo read enable signal
    wire                      syn_off0_vs;
    wire                      syn_off0_hs;

    wire                      off0_syn_de  ;
    wire [RD_VIDEO_WIDTH-1:0] off0_syn_data;

    wire[15:0]                      cmos_16bit_data;
    wire                            cmos_16bit_clk;
    wire                            cmos_16bit_wr;

    wire[15:0] 						write_data;

    wire[9:0]                       lut_index;
    wire[31:0]                      lut_data;
    wire i2c_done;
    wire i2c_err;

    assign cmos_pwdn = 1'b0;
//    assign cmos_rst_n = 1'b1;
    assign cmos_rst_n = cmos_reset;
    assign hpd_en = 1;



    //generate the CMOS sensor clock and the SDRAM controller, I2C controller clock
    wire [1:0]  DDRPLL_MDR_OPC;
    wire        DDRPLL_MDR_AINC;
    wire [7:0]  DDRPLL_MDR_DO;
    wire [7:0]  DDRPLL_MDR_DI;
    DDR_PLL Gowin_PLL_m0(
    	.clkin                     (clk                         ),
    	.clkout0                   (cmos_clk 	              	),
        .clkout1                   (aux_clk 	              	),
        .clkout2                   (memory_clk 	              	),
    	.lock 					   (DDR_pll_lock 				),
        .reset                     (~rst_n ),
            //25/60K Extra Control
        .mdclk                     (clk),
        .mdopc                     (DDRPLL_MDR_OPC), //input [1:0] mdopc
        .mdainc                    (DDRPLL_MDR_AINC), //input mdainc
        .mdrdo                     (DDRPLL_MDR_DO), //output [7:0] mdrdo
        .mdwdi                     (DDRPLL_MDR_DI) //input [7:0] mdwdi
	);

    // Reset for camera sensor
    reg [31:0] cmos_reset_delay_cnt;
    reg cmos_reset;
    reg cmos_start_config;
    always@(posedge clk or negedge rst_n)
    begin
        if(!rst_n)
        begin
            cmos_reset_delay_cnt <= 0;
            cmos_reset <= 0;
            cmos_start_config <= 0;
        end else begin
            if(cmos_reset_delay_cnt == 32'd3_000_000)
            begin
                cmos_reset_delay_cnt <= cmos_reset_delay_cnt;
                cmos_reset <= 1'b1;
                cmos_start_config <= 1'b1;
            end else if(cmos_reset_delay_cnt == 32'd100_000)
            begin
                cmos_reset_delay_cnt <= cmos_reset_delay_cnt + 1;
                cmos_reset <= 1'b1;
                cmos_start_config <= 1'b0;
            end else begin
                cmos_reset_delay_cnt <= cmos_reset_delay_cnt + 1;
                cmos_reset <= cmos_reset;
                cmos_start_config <= cmos_start_config;
            end
            
        end
    end

    //configure look-up table
    // --- I2C Controller for Camera 1 ---
    wire [9:0]  lut_index_cam1;
    wire [31:0] lut_data_cam1;
    wire        i2c_done_cam1;
    wire        i2c_err_cam1;

    lut_final_transplant #(
        .HActive(12'd1280), .VActive(12'd720), .HTotal(13'd1892), .VTotal(13'd740), .USE_4vs3_frame("false")
    ) lut_final_transplant_m0 (
        .lut_index(lut_index_cam1),
        .lut_data(lut_data_cam1)
    );

    i2c_config i2c_config_m0 (
        .rst           (~cmos_start_config),
        .clk           (clk),
        .clk_div_cnt   (16'd1000),
        .i2c_addr_2byte(1'b1),
        .lut_index     (lut_index_cam1),
        .lut_dev_addr  (lut_data_cam1[31:24]), // 取查找表高 8 位，保留 index 252 的 0xFF 终止哨兵
        .lut_reg_addr  (lut_data_cam1[23:8]),
        .lut_reg_data  (lut_data_cam1[7:0]),
        .error         (i2c_err_cam1),
        .done          (i2c_done_cam1),
        .i2c_scl       (cmos_scl), // Connects to Cam 1 SCL
        .i2c_sda       (cmos_sda)  // Connects to Cam 1 SDA
    );

    // --- I2C Controller for Camera 2 ---
    wire [9:0]  lut_index_cam2;
    wire [31:0] lut_data_cam2;
    wire        i2c_done_cam2;
    wire        i2c_err_cam2;

    lut_final_transplant #(
        .HActive(12'd1280), .VActive(12'd720), .HTotal(13'd1892), .VTotal(13'd740), .USE_4vs3_frame("false")
    ) lut_final_transplant_m1 ( // New instance name
        .lut_index(lut_index_cam2),
        .lut_data(lut_data_cam2)
    );

    i2c_config i2c_config_m1 ( // New instance name
        .rst           (~cmos_start_config),
        .clk           (clk),
        .clk_div_cnt   (16'd1000),
        .i2c_addr_2byte(1'b1),
        .lut_index     (lut_index_cam2),
        .lut_dev_addr  (lut_data_cam2[31:24]), // 取查找表高 8 位，保留 index 252 的 0xFF 终止哨兵
        .lut_reg_addr  (lut_data_cam2[23:8]),
        .lut_reg_data  (lut_data_cam2[7:0]),
        .error         (i2c_err_cam2),
        .done          (i2c_done_cam2),
        .i2c_scl       (cmos2_scl), // Connects to Cam 2 SCL
        .i2c_sda       (cmos2_sda)  // Connects to Cam 2 SDA
    );

    // --- I2C Controller for Camera 3 ---
    wire [9:0]  lut_index_cam3;
    wire [31:0] lut_data_cam3;
    wire        i2c_done_cam3;
    wire        i2c_err_cam3;

    lut_final_transplant #(
        .HActive(12'd1280), .VActive(12'd720), .HTotal(13'd1892), .VTotal(13'd740), .USE_4vs3_frame("false")
    ) lut_final_transplant_m2 ( 
        .lut_index(lut_index_cam3),
        .lut_data(lut_data_cam3)
    );

    i2c_config i2c_config_m2 (
        .rst           (~cmos_start_config),
        .clk           (clk),
        .clk_div_cnt   (16'd1000),
        .i2c_addr_2byte(1'b1),
        .lut_index     (lut_index_cam3),
        .lut_dev_addr  (lut_data_cam3[31:24]), // 取查找表高 8 位，保留 index 252 的 0xFF 终止哨兵
        .lut_reg_addr  (lut_data_cam3[23:8]),
        .lut_reg_data  (lut_data_cam3[7:0]),
        .error         (i2c_err_cam3),
        .done          (i2c_done_cam3),
        .i2c_scl       (cmos3_scl), // Connects to Cam 3 SCL
        .i2c_sda       (cmos3_sda)  // Connects to Cam 3 SDA
    );


    assign i2c_done = i2c_done_cam1 & i2c_done_cam2 & i2c_done_cam3;
    assign i2c_err  = i2c_err_cam1 | i2c_err_cam2 | i2c_err_cam3;
    
    //CMOS sensor 8bit data is converted to 16bit data
    cmos_8_16bit cmos_8_16bit_m0(
    	.rst                        (~rst_n                   ),
    	.pclk                       (cmos_pclk                ),
    	.pdata_i                    (cmos_db                  ),
    	.de_i                       (cmos_href                ),
    	.pdata_o                    (cmos_16bit_data          ),
    	.hblank                     (cmos_16bit_wr            ),
    	.de_o                       (cmos_16bit_clk           )
    );
// 二号摄像头
    wire [15:0] cmos2_16bit_data;
    wire        cmos2_16bit_wr;
    wire        cmos2_16bit_clk;

    // [修复] VSYNC 跨时钟域：直接同步"电平"到 cam1 像素时钟域，再在目的域内做边沿检测。
    // 旧实现先在 cam2 域取沿成 1 拍窄脉冲、再打两拍跨域，窄脉冲会被异步采样漏掉
    // （约五成概率），导致行指针偶发整帧不复位、过渡区内容逐帧跳变。
    // 电平同步 + 目的域取沿只可能延迟、不会丢失。
    reg  cam2_vsync_sync0, cam2_vsync_sync1;
    wire cam2_vsync_synced_pulse;

    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n) begin
            cam2_vsync_sync0 <= 1'b0;
            cam2_vsync_sync1 <= 1'b0;
        end else begin
            cam2_vsync_sync0 <= cmos2_vsync;
            cam2_vsync_sync1 <= cam2_vsync_sync0;
        end
    end
    assign cam2_vsync_synced_pulse = cam2_vsync_sync0 & ~cam2_vsync_sync1;

    cmos_8_16bit cmos_8_16bit_m1( // New instance name
    	.rst                        (~rst_n),
    	.pclk                       (cmos2_pclk),
    	.pdata_i                    (cmos2_db),
    	.de_i                       (cmos2_href),
    	.pdata_o                    (cmos2_16bit_data),
    	.hblank                     (cmos2_16bit_wr),
    	.de_o                       (cmos2_16bit_clk)
    );

    wire [15:0] write_data_cam2;
    assign write_data_cam2 = {cmos2_16bit_data[15:11], cmos2_16bit_data[10:5], cmos2_16bit_data[4:0]};
    
    // --- 三号摄像头数据转换和同步 ---
    wire [15:0] cmos3_16bit_data;
    wire        cmos3_16bit_wr;
    wire        cmos3_16bit_clk;

    cmos_8_16bit cmos_8_16bit_m2( // New instance name
        .rst                        (~rst_n),
        .pclk                       (cmos3_pclk),
        .pdata_i                    (cmos3_db),
        .de_i                       (cmos3_href),
        .pdata_o                    (cmos3_16bit_data),
        .hblank                     (cmos3_16bit_wr),
        .de_o                       (cmos3_16bit_clk)
    );

    // 定义连接FIFO的信号 
    wire [15:0] cam2_fifo_rdata;
    wire        cam2_fifo_empty;
    wire        cam2_fifo_full;
    wire        cam2_fifo_wr_en;
    wire        cam2_fifo_rd_en; 
    assign cam2_fifo_wr_en = cmos2_16bit_wr;
    wire cam2_fifo_reset = ~rst_n;


    // [修复] 同 cam2：电平同步 + 目的域取沿，消除 VSYNC 跨域漏采
    reg  cam3_vsync_sync0, cam3_vsync_sync1;
    wire cam3_vsync_synced_pulse;

    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n) begin
            cam3_vsync_sync0 <= 1'b0;
            cam3_vsync_sync1 <= 1'b0;
        end else begin
            cam3_vsync_sync0 <= cmos3_vsync;
            cam3_vsync_sync1 <= cam3_vsync_sync0;
        end
    end
    assign cam3_vsync_synced_pulse = cam3_vsync_sync0 & ~cam3_vsync_sync1;


    fifo_cam2 u_cam2_fifo (
        // 写端口
        .WrClk(cmos2_16bit_clk), 
        .WrEn(cam2_fifo_wr_en),  
        .Data(write_data_cam2), 
        .Full(cam2_fifo_full), 

        // 读端口
        .RdClk(cmos_16bit_clk), 
        .RdEn(cam2_fifo_rd_en),
        .Q(cam2_fifo_rdata), 
        .Empty(cam2_fifo_empty),
        .Reset(cam2_fifo_reset),
        .Almost_Full(),
        .Almost_Empty()
    );

    wire [15:0] write_data_cam3;
    assign write_data_cam3 = {cmos3_16bit_data[15:11], cmos3_16bit_data[10:5], cmos3_16bit_data[4:0]};

    wire [15:0] cam3_fifo_rdata;
    wire        cam3_fifo_empty;
    wire        cam3_fifo_full;
    wire        cam3_fifo_wr_en = cmos3_16bit_wr;
    wire        cam3_fifo_rd_en;
    wire        cam3_fifo_reset = ~rst_n;

    fifo_cam2 u_cam3_fifo ( 
        .WrClk(cmos3_16bit_clk),
        .WrEn(cmos3_16bit_wr),
        .Data(write_data_cam3),
        .Full(cam3_fifo_full),
        .RdClk(cmos_16bit_clk),
        .RdEn(cam3_fifo_rd_en),
        .Q(cam3_fifo_rdata),
        .Empty(cam3_fifo_empty),
        .Reset(cam3_fifo_reset),
        .Almost_Full(),
        .Almost_Empty()
    );

    //=========================================================================
    // [2026-11-02 修] DOA 结果线必须在**模块级、且在使用之前**声明
    //
    //   坑：pan_ctrl 例化在 `generate if(USE_TPG=="false")` 之内（约 top.v:1032），
    //   audio_top 例化在 generate 之外（约 :1326）。这两个位置都要看到同一组线。
    //
    //   · 只在 generate 内声明 → audio_top 侧变成模块级隐式 1 位网，pan_ctrl 侧悬空；
    //   · 声明在模块级但写在 generate 之后 → Verilog 要求先声明后使用，
    //     Gowin 报 `Undeclared symbol 'doa_bearing', assumed default net type 'wire'`
    //     仍然按 1 位处理。
    //   ⇒ 唯一正确位置：模块级、且在 generate 之前（就是这里）。
    //=========================================================================
    wire signed [17:0] doa_bearing;    // Q2.16，32768 == 90°
    wire [3:0]         doa_sector;     // 0..11，每扇区 30°
    wire [7:0]         doa_conf;       // 0..255
    wire               doa_valid;      // 单拍：每块解算完成（≈84 ms）

    // ---- 状态指示灯（放在此处以确保所有被引用信号均已声明）----
    //  上电约 60ms 后三路 I2C 配置开始；[3:1] 全灭 = 三路配置均无错
    // [2026-11-02 诊断] state_led[3:1] 临时改为 DOA 联动指示（见 generate 内）
    //   原含义为三路相机 I2C 配置失败。要恢复只需把下面三行注释打开、
    //   并把 generate 内那三条 assign 删掉。
    // assign state_led[3] = i2c_err_cam1;   // 相机 1 I2C 配置失败
    // assign state_led[2] = i2c_err_cam2;   // 相机 2 I2C 配置失败
    // assign state_led[1] = i2c_err_cam3;   // 相机 3 I2C 配置失败
    assign state_led[4] = cam2_fifo_full | cam3_fifo_full;   // cam2/cam3 FIFO 溢出
    assign state_led[0] = ~init_calib_complete               // 系统未就绪
                        | ~DDR_pll_lock
                        | ~TMDS_DDR_pll_lock;

    wire out_de;
    wire [9:0] lcd_x,lcd_y;
    //assign write_data = {cmos_16bit_data[7:0], cmos_16bit_data[15:8]};
    assign write_data = {cmos_16bit_data[15:11], cmos_16bit_data[10:5], cmos_16bit_data[4:0]};
    vga_timing #(
        .H_ACTIVE(16'd1280), 
        .H_FP(16'd110),
        .H_SYNC(16'd40),
        .H_BP(16'd220),
        .V_ACTIVE(16'd720),
        .V_FP(16'd5),
        .V_SYNC(16'd5),
        .V_BP(16'd20), 	
        .HS_POL(1'b1),   	
        .VS_POL(1'b1)
    ) vga_timing_m0(
        .clk (video_clk),
        .rst (~rst_n),
        .active_x(lcd_x),
        .active_y(lcd_y),
        .hs(syn_off0_hs),
        .vs(syn_off0_vs),
        .de(out_de)
    );
    

    //Test pattern generate
    ///--------------------------
    wire        tp0_vs_in  ;
    wire        tp0_hs_in  ;
    wire        tp0_de_in ;
    wire [ 7:0] tp0_data_r;
    wire [ 7:0] tp0_data_g;
    wire [ 7:0] tp0_data_b;

    generate if(USE_TPG == "true")         
    begin
        testpattern testpattern_inst_1280
        (
            .I_pxl_clk   (video_clk    ),//pixel clock
            .I_rst_n     (rst_n        ),//low active 
            .I_mode      (3'b000       ),//data select
            .I_single_r  (8'd255       ),
            .I_single_g  (8'd255       ),
            .I_single_b  (8'd255       ),                  //800x600    //1024x768   //1280x720   //1920x1080 
            .I_h_total   (12'd1650     ),//hor total time  // 12'd1056  // 12'd1344  // 12'd1650  // 12'd2200
            .I_h_sync    (12'd40       ),//hor sync time   // 12'd128   // 12'd136   // 12'd40    // 12'd44  
            .I_h_bporch  (12'd220      ),//hor back porch  // 12'd88    // 12'd160   // 12'd220   // 12'd148 
            .I_h_res     (12'd1280     ),//hor resolution  // 12'd800   // 12'd1024  // 12'd1280  // 12'd1920
            .I_v_total   (12'd750      ),//ver total time  // 12'd628   // 12'd806   // 12'd750   // 12'd1125 
            .I_v_sync    (12'd5        ),//ver sync time   // 12'd4     // 12'd6     // 12'd5     // 12'd5   
            .I_v_bporch  (12'd20       ),//ver back porch  // 12'd23    // 12'd29    // 12'd20    // 12'd36  
            .I_v_res     (12'd720      ),//ver resolution  // 12'd600   // 12'd768   // 12'd720   // 12'd1080 
            .I_hs_pol    (1'b1         ),//0,负极性;1,正极性
            .I_vs_pol    (1'b1         ),//0,负极性;1,正极性
            .O_de        (tp0_de_in    ),   
            .O_hs        (tp0_hs_in    ),
            .O_vs        (tp0_vs_in    ),
            .O_data_r    (tp0_data_r   ),   
            .O_data_g    (tp0_data_g   ),
            .O_data_b    (tp0_data_b   )
        );
    end
    endgenerate
    
// MODIFIED: Input data for DDR Buffer with a MUX to select camera
wire fb_vin_clk;
wire fb_vin_vsync;
wire [15:0] fb_vin_data;
wire fb_vin_de;

generate if(USE_TPG == "false")
begin //CMOS DATA

    reg [15:0] cam1_line_buffer [0:1279]; // 摄像头1(中央)的行缓冲
    reg [10:0] line_buffer_wr_ptr_cam1;   

    reg [15:0] cam2_line_buffer [0:1279]; // 摄像头2(左侧)的行缓冲
    reg [10:0] line_buffer_wr_ptr_cam2;   

    reg [15:0] cam3_line_buffer [0:1279]; // 摄像头3(右侧)的行缓冲
    reg [10:0] line_buffer_wr_ptr_cam3;   

    // --- 控制数据写入各自的行缓冲 ---

    //---------------------------------------------------------------------
    // [2026-11-01 U3 修复] cam1(U3) 的 VSYNC 改为与 cam2/cam3 完全相同的处理方式
    //
    //   原实现：`else if (cmos_vsync) ptr <= 0;` —— 直接采样**原始电平**、且未同步。
    //   U4/U5 早已改成"两级同步 + 目的域内边沿检测"（见 cam2/cam3 的实现与注释）；
    //   唯独 U3 仍是裸采样。VSYNC 相对 cmos_16bit_clk(=pclk/2) 是异步的，裸采样会：
    //     · 亚稳态 ⇒ 行指针复位时刻逐帧抖动 ⇒ 每帧行起点不定 ⇒ **横向条纹/栅格化**；
    //     · 电平复位期间（垂直消隐若干行）持续清零，若传感器在 VSYNC 期间仍有 href，
    //       这些行数据会被全部丢到列 0。
    //   修法：2 级同步 + 上升沿单拍脉冲，与 U4/U5 一致。
    //---------------------------------------------------------------------
    reg  cam1_vsync_sync0, cam1_vsync_sync1;
    wire cam1_vsync_synced_pulse;

    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n) begin
            cam1_vsync_sync0 <= 1'b0;
            cam1_vsync_sync1 <= 1'b0;
        end else begin
            cam1_vsync_sync0 <= cmos_vsync;
            cam1_vsync_sync1 <= cam1_vsync_sync0;
        end
    end
    assign cam1_vsync_synced_pulse = cam1_vsync_sync0 & ~cam1_vsync_sync1;

    // 摄像头1(中央)行缓冲写入
    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n) begin
            line_buffer_wr_ptr_cam1 <= 11'd0;
        end
        else if (cam1_vsync_synced_pulse) begin
            line_buffer_wr_ptr_cam1 <= 11'd0;
        end
        else if (cmos_16bit_wr) begin
            cam1_line_buffer[line_buffer_wr_ptr_cam1] <= write_data;
            line_buffer_wr_ptr_cam1 <= (line_buffer_wr_ptr_cam1 == 1279) ? 11'd0 : line_buffer_wr_ptr_cam1 + 1;
        end
    end

    // 摄像头2(左侧)行缓冲写入
    // 注意：必须"非空即读"（全速排空）。FIFO 仅 512 深（DEPTH=9），不足半行；
    // 若把读出限制在 cam1 有效像素窗口内（每行 1280 个 12MHz 周期），则每行读出上限
    // 恰等于写入量 1280 words（零余量），而消隐期 612 个周期内可积压 612 words
    // > 512 → FIFO 溢出丢数、cam2 图像错乱（cam1 不经 FIFO，故不受影响）。
    wire read_from_fifo2 = !cam2_fifo_empty;
    assign cam2_fifo_rd_en = read_from_fifo2;

    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if(!rst_n) begin
            line_buffer_wr_ptr_cam2 <= 11'd0;
        end
        else if (cam2_vsync_synced_pulse) begin
            line_buffer_wr_ptr_cam2 <= 11'd0;
        end
        else if (read_from_fifo2) begin
            cam2_line_buffer[line_buffer_wr_ptr_cam2] <= cam2_fifo_rdata;
            line_buffer_wr_ptr_cam2 <= (line_buffer_wr_ptr_cam2 == 1279) ? 11'd0 : line_buffer_wr_ptr_cam2 + 1;
        end
    end

    // 摄像头3(右侧)行缓冲写入
    // 同 cam2：必须全速排空，不得门控（见上方说明）
    wire read_from_fifo3 = !cam3_fifo_empty;
    assign cam3_fifo_rd_en = read_from_fifo3;
    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if(!rst_n) begin
            line_buffer_wr_ptr_cam3 <= 11'd0;
        end
        else if (cam3_vsync_synced_pulse) begin 
            line_buffer_wr_ptr_cam3 <= 11'd0;
        end
        else if (read_from_fifo3) begin
            cam3_line_buffer[line_buffer_wr_ptr_cam3] <= cam3_fifo_rdata;
            line_buffer_wr_ptr_cam3 <= (line_buffer_wr_ptr_cam3 == 1279) ? 11'd0 : line_buffer_wr_ptr_cam3 + 1;
        end
    end

    // 像素计数器
    reg [10:0] pixel_counter, pixel_counter_d1, pixel_counter_d2;
    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_counter <= 0; pixel_counter_d1 <= 0; pixel_counter_d2 <= 0;
        end else if (cmos_16bit_wr) begin
            pixel_counter <= (pixel_counter == 1279) ? 0 : pixel_counter + 1;
            pixel_counter_d1 <= pixel_counter;
            pixel_counter_d2 <= pixel_counter_d1;
        end else begin
            pixel_counter <= 0; 
            pixel_counter_d1 <= 0; 
            pixel_counter_d2 <= 0;
        end
    end
    
    // 按键消抖 
    reg [19:0] debounce_cnt_left;
    reg [19:0] debounce_cnt_right;
    reg key_left_reg, key_right_reg;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            debounce_cnt_left <= 20'd0;
            debounce_cnt_right <= 20'd0;
            key_left_reg <= 1'b1;
            key_right_reg <= 1'b1;
        end else begin
            if (key_left == 1'b0) begin 
                if (debounce_cnt_left < 20'd1_000_000) 
                    debounce_cnt_left <= debounce_cnt_left + 1;
                else
                    key_left_reg <= 1'b0;
            end else begin 
                debounce_cnt_left <= 20'd0;
                key_left_reg <= 1'b1;
            end
            if (key_right == 1'b0) begin 
                if (debounce_cnt_right < 20'd1_000_000)
                    debounce_cnt_right <= debounce_cnt_right + 1;
                else
                    key_right_reg <= 1'b0;
            end else begin 
                debounce_cnt_right <= 20'd0;
                key_right_reg <= 1'b1;
            end
        end
    end

    localparam IMG_WIDTH       = 1280; 
    //---------------------------------------------------------------------
    // 重叠带宽度 = 256 px。**这个值是本机型调好的，不要凭理论推倒它。**
    //
    // 2026-11-01 曾按"120° 镜头 + 120° 间隔 ⇒ 重叠≈0"把这里改成 32（每颗用满 1280 px），
    // 结果上板后拼接明显变差 —— 因为实测三颗之间**确实存在约 256 px 的重叠**：
    // 改成全宽后，重叠区的内容在两颗相机里各出现一次、相隔 256 px，
    // 而代码只融 32 px ⇒ 接缝处变成"景物重复/错位"，比重影更刺眼。
    //
    // 结论：**相邻相机的真实重叠 ≈ 256 px（30°）；每颗相机参与拼接的是中间 1024 px。**
    // 重影/发虚的根因不是重叠宽度，而是重叠区两侧的**几何尺度不一致**（见 docs/FISHEYE_STITCH.md）。
    //---------------------------------------------------------------------
    localparam OVERLAP_WIDTH   = 256; 
    localparam NON_OVERLAP     = IMG_WIDTH - OVERLAP_WIDTH; 
    // 全景为闭环：cam3 尾与 cam2 头之间也保留同样的融合带（否则那一处是硬切换）。
    localparam PANORAMA_WIDTH  = IMG_WIDTH * 3 - OVERLAP_WIDTH * 3; // 3072 = 360°
    // 核心计算公式：OTATION_PERIOD = (clk_frequency * ROTATION_STEP) / Desired_Speed_In_Pixels_Per_Second
    localparam ROTATION_STEP = 8; 
    localparam ROTATION_PERIOD = 416_667;

    //---------------------------------------------------------------------
    // 自动接管通路（DOA × 人脸 × pan_ctrl）
    //
    //   数据流：
    //     audio_top.doa_*  ──┐
    //                         ├─→ pan_ctrl ─→ auto_offset / auto_active
    //     显示侧像素流 ─→ face_det_stream ─┘        │
    //       (off0_syn_*)      │ face_cx/conf/tgl     ▼
    //                         └─(toggle 跨域)─→  MUX ─→ horizontal_offset
    //
    //   注意：face_det_stream 吃的是**显示侧**像素流，与观众看到的完全同源，
    //   所以输出的块坐标 ×8 就是屏幕像素，不需要任何换算/补偿。
    //---------------------------------------------------------------------
    wire              auto_active;            // 1 = 自动接管取景窗
    wire [12:0]       auto_offset;            // 自动取景窗位置

    // DOA 四根线声明在模块级（audio_top 例化在 generate 外，见文件末尾）

    // 人脸检测结果（视频域输出，块坐标）
    wire [7:0]         face_cx, face_cy, face_w, face_h, face_conf;
    wire               face_tgl;

    // 按键（低有效已消抖 → 高有效"有键按下"）
    wire               any_key = ~key_left_reg | ~key_right_reg;

    // pan_ctrl 的调试输出（当前相机 / 标定后扇区），仅供观察
    wire [1:0]         dbg_cam;
    wire [3:0]         dbg_sect;

    //---------------------------------------------------------------------
    // [2026-11-02] 按键平移 —— **原始逻辑一字不改**，只把目标寄存器改名
    //   manual_offset。自动接管（DOA / 人脸）走另一条独立通路，最后由 MUX 二选一。
    //
    //   这是"新功能只能旁路叠加"的落地方式：即使自动模块全错、auto_active 恒 0，
    //   按键仍然是同事原始版本的行为（8 px / 8.33 ms = 960 px/s，自足的 clk 域计数，
    //   不依赖 vsync/DOA/人脸任何外部信号）。
    //---------------------------------------------------------------------
    reg signed [12:0] manual_offset;          // 手动取景窗位置（原 horizontal_offset）
    reg [20:0] rotation_speed_cnt;
    reg signed [12:0] horizontal_offset;      // 最终取景窗位置（MUX 输出，见下）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            manual_offset <= 0;
            rotation_speed_cnt <= 0;
        end else if (auto_active) begin
            // 自动接管期间：手动位置**跟随当前画面位置**
            //   ⇒ 一旦按键把 auto_active 拉低，MUX 切回 manual_offset 时两者相等，
            //     画面不会突然跳回上一次手动停留的地方。
            manual_offset <= horizontal_offset;
            rotation_speed_cnt <= 0;
        end else begin
            if (key_left_reg == 1'b0) begin
                if (rotation_speed_cnt < ROTATION_PERIOD) begin
                    rotation_speed_cnt <= rotation_speed_cnt + 1;
                end else begin
                    rotation_speed_cnt <= 0;
                    if (manual_offset < ROTATION_STEP)
                        manual_offset <= manual_offset - ROTATION_STEP + PANORAMA_WIDTH;
                    else
                        manual_offset <= manual_offset - ROTATION_STEP;
                end
            end else if (key_right_reg == 1'b0) begin
                if (rotation_speed_cnt < ROTATION_PERIOD) begin
                    rotation_speed_cnt <= rotation_speed_cnt + 1;
                end else begin
                    rotation_speed_cnt <= 0;
                    if (manual_offset >= PANORAMA_WIDTH - ROTATION_STEP)
                        manual_offset <= manual_offset + ROTATION_STEP - PANORAMA_WIDTH;
                    else
                        manual_offset <= manual_offset + ROTATION_STEP;
                end
            end else begin
                rotation_speed_cnt <= 0;
            end
        end
    end

    //=====================================================================
    // [2026-11-02 新增] 自动接管：节拍 / MUX / 帧锁存
    //
    //  一、为什么节拍用**本地计数**而不是相机 VSYNC
    //  ----------------------------------------------------------------
    //  拼接是逐行进行的（cmos_16bit_clk 域，一帧 720 行 ≈ 66.7 ms）。
    //  取景窗偏移若在同一帧中途变化，先拼的行与后拼的行取自不同取景位置
    //  ⇒ 转动时画面上下错位（撕裂）。所以偏移必须"每帧只变一次"。
    //
    //  但"帧边界"绝不能只依赖 cam1 的 VSYNC：一旦该信号丢失，节拍就永久停住，
    //  手动与自动会**同时**失效，而画面上完全看不出来（行指针不复位也会每
    //  1280 像素自然回绕，而行宽正好 1280）。所以这里用独立的本地计数器：
    //  50 MHz × 66.7 ms 与 pclk/2 × 700,040 拍，各自成帧，零外部依赖。
    //=====================================================================
    localparam integer SOFT_TICK_CYC  = 3_333_333;   // clk 域：50 MHz × 66.7 ms
    localparam integer SOFT_LATCH_CYC =   700_000;   // cmos 域：pclk/2 × 700,000 ≈ 1 帧

    // ---- (1) clk 域：平移节拍（pan_ctrl 的唯一更新时间点）----
    reg [22:0] soft_cnt;
    wire       step_tick = (soft_cnt >= SOFT_TICK_CYC[22:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          soft_cnt <= 23'd0;
        else if (step_tick)  soft_cnt <= 23'd0;
        else                 soft_cnt <= soft_cnt + 23'd1;
    end

    // ---- (2) 取景窗最终值：自动 / 手动 二选一 ----
    //   ★ 唯一的仲裁点。auto_active=1 才用自动输出；否则完全等同原始行为。
    //     按键会当拍把 auto_active 拉低（pan_ctrl 内 act_cnt 清零），所以
    //     "按键优先"不需要额外的优先级逻辑。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) horizontal_offset <= 13'sd0;
        else        horizontal_offset <= auto_active ? auto_offset : manual_offset[12:0];
    end

    // ---- (3) cmos 域：取景窗帧锁存 —— **VSYNC 优先，本地计数兜底** ----
    //
    //  为什么必须用真实 VSYNC 而不是本地计数（2026-11-03 修「画面整体倾斜」）：
    //    拼接是**逐行**进行的。取景窗偏移只有在"一行都还没开始"的帧边界改变，
    //    整帧才取自同一个取景位置；一旦锁存时刻落在帧中途，同一帧的上半屏与
    //    下半屏就来自不同偏移 ⇒ 画面被斜着切开（观感就是"整体向左倾斜"）。
    //
    //    之前用 SOFT_LATCH_CYC = 700_000 拍作为帧长估计，但它取决于
    //    cmos_16bit_clk 的实际频率（pclk/2，随 OV5640 PLL 配置变化）。
    //    估计值偏大/偏小都会让锁存点逐帧漂移，周期性落进帧中间 ⇒ 间歇性斜切。
    //    改用 cam1 的 vsync 同步脉冲（cam1_vsync_synced_pulse）就与真实帧边界
    //    严格对齐，不存在漂移。
    //
    //  为什么仍保留本地计数兜底：
    //    如果 cam1 的 VSYNC 丢失，只用 vsync 会让偏移永久冻结，转动失效。
    //    这里是"vsync 活着就用它，死了退化成本地计数"——功能优先，
    //    退化时可能重新出现轻微斜切，但至少画面还能转。
    //    ★ 注意：pan 的步进节拍（step_tick）始终是本地计数，不受此处影响，
    //      所以 vsync 丢失也不会让手动/自动停摆。
    reg [23:0] vs_gap_c;
    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n)                          vs_gap_c <= 24'd0;
        else if (cam1_vsync_synced_pulse)    vs_gap_c <= 24'd0;
        else if (vs_gap_c != 24'hFFFFFF)     vs_gap_c <= vs_gap_c + 24'd1;
    end
    wire vs_ok_c = (vs_gap_c < 24'd2_000_000);      // ≈166 ms @12 MHz 内见过 vsync

    reg [22:0] soft_cnt_c;
    wire       soft_latch = (soft_cnt_c >= SOFT_LATCH_CYC[22:0]);

    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n)             soft_cnt_c <= 23'd0;
        else if (soft_latch)    soft_cnt_c <= 23'd0;
        else                    soft_cnt_c <= soft_cnt_c + 23'd1;
    end

    wire latch_en = vs_ok_c ? cam1_vsync_synced_pulse : soft_latch;

    reg signed [12:0] offset_frame;           // 本帧使用的取景窗左边界（帧内恒定）

    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n)          offset_frame <= 13'sd0;
        else if (latch_en)   offset_frame <= horizontal_offset_sync1;
    end


    reg signed [12:0] horizontal_offset_sync0;
    reg signed [12:0] horizontal_offset_sync1;

    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if (!rst_n) begin
            horizontal_offset_sync0 <= 0;
            horizontal_offset_sync1 <= 0;
        end else begin
            horizontal_offset_sync0 <= horizontal_offset;
            horizontal_offset_sync1 <= horizontal_offset_sync0;
        end
    end    



    localparam CAM1_START_PANO = NON_OVERLAP;
    localparam CAM3_START_PANO = NON_OVERLAP + NON_OVERLAP;
    localparam BLEND1_START = CAM1_START_PANO;
    localparam BLEND1_END   = CAM1_START_PANO + OVERLAP_WIDTH;
    localparam BLEND2_START = CAM3_START_PANO;
    localparam BLEND2_END   = CAM3_START_PANO + OVERLAP_WIDTH;
    // [新增] 回绕缝融合带：cam3 尾 256 列（列 1024..1279）↔ cam2 头 256 列（列 0..255）
    localparam CAM3_TAIL    = IMG_WIDTH - OVERLAP_WIDTH; // 1024

    // --- 流水线数据路径寄存器 ---
    // Stage 1
    reg [13:0] r_mult_sum_s1;
    reg [14:0] g_mult_sum_s1;
    reg [13:0] b_mult_sum_s1;
    reg [15:0] direct_data_s1;
    // Stage 2
    reg [15:0] stitched_data_s2;

    // --- [FIX] 流水线控制路径寄存器 ---
    // Stage 1
    reg is_blending_s1;
    reg vsync_s1;
    reg de_s1;
    // Stage 2
    reg vsync_s2;
    reg de_s2;

    assign fb_vin_data   = stitched_data_s2;
    assign fb_vin_de     = de_s2;
    assign fb_vin_vsync  = vsync_s2;
    assign fb_vin_clk    = cmos_16bit_clk; // 时钟直连

    reg [12:0] pano_coord_s0;
    reg [15:0] data_left_s0, data_right_s0;
    reg [7:0]  alpha_s0;
    reg [7:0]  w1, w2;
    reg        is_blending_region_s0;

    //---------------------------------------------------------------------
    // [2026-11-01 v6] 鱼眼水平重映射（180° 广角 M12 镜头校正）
    //
    // 模型：M12 180° 鱼眼 FOV 为【对角线】口径，投影近似等立体角 r=2f·sin(θ/2)
    //      ⇒ 水平 FOV = 152.2°（三条独立证据支持，见 docs/OV5640_180_M12_CONFIG.md）。
    // 重映射：全景列 x（相机本地，光轴 640）对应世界角 φ=(x-640)×0.1172°，
    //      采样源列 src = 640 + 1038.5·sin(φ/2)。与恒等映射最大偏差仅 14 px。
    //
    // 想关掉校正回到恒等：重新生成 LUT（tools/fisheye_remap_gen.py --proj identity）。
    //---------------------------------------------------------------------
    reg  [10:0] ix_a_s0, ix_b_s0;         // A/B 路相机本地列（0..1279）
    wire [10:0] src_a_w, src_b_w;         // 重映射后的源列

    fisheye_remap_lut u_remap_a (.x(ix_a_s0), .src(src_a_w));
    fisheye_remap_lut u_remap_b (.x(ix_b_s0), .src(src_b_w));

    wire [5:0] blended_r_s1; // R通道(14位) >> 8 = 6位
    wire [6:0] blended_g_s1; // G通道(15位) >> 8 = 7位
    wire [5:0] blended_b_s1; // B通道(14位) >> 8 = 6位

    assign blended_r_s1 = r_mult_sum_s1 >> 8;
    assign blended_g_s1 = g_mult_sum_s1 >> 8;
    assign blended_b_s1 = b_mult_sum_s1 >> 8;

    always @(posedge cmos_16bit_clk or negedge rst_n) begin
        if(!rst_n) begin
            r_mult_sum_s1 <= 0; g_mult_sum_s1 <= 0; b_mult_sum_s1 <= 0;
            direct_data_s1 <= 0;
            stitched_data_s2 <= 0;
            is_blending_s1 <= 0;
            vsync_s1 <= 0; vsync_s2 <= 0;
            de_s1 <= 0; de_s2 <= 0;
        end else begin
            if (is_blending_s1) begin
                stitched_data_s2 <= { blended_r_s1[4:0], blended_g_s1[5:0], blended_b_s1[4:0] };
            end else begin
                stitched_data_s2 <= direct_data_s1;
            end
            vsync_s2 <= vsync_s1;
            de_s2    <= de_s1;
            is_blending_s1 <= is_blending_region_s0;
            vsync_s1       <= cmos_vsync;
            de_s1          <= cmos_16bit_wr;
            if (is_blending_region_s0) begin
                w1 = 255 - alpha_s0;
                w2 = alpha_s0;
                r_mult_sum_s1 <= (data_left_s0[15:11] * w1) + (data_right_s0[15:11] * w2);
                g_mult_sum_s1 <= (data_left_s0[10:5]  * w1) + (data_right_s0[10:5]  * w2);
                b_mult_sum_s1 <= (data_left_s0[4:0]   * w1) + (data_right_s0[4:0]   * w2);
            end
            pano_coord_s0 = offset_frame + pixel_counter_d2;   // 帧内恒定（消除转动撕裂）
            if (pano_coord_s0 >= PANORAMA_WIDTH) begin
                pano_coord_s0 = pano_coord_s0 - PANORAMA_WIDTH;
            end

            //-----------------------------------------------------------------
            // [2026-11-01 v5] 恢复恒等映射（经验基线），并记录为什么：
            //
            //  本轮曾按用户口径（120°鱼眼→120°无畸变→130°→180°）三次重推几何，
            //  最后一次按 180° 等距鱼眼加了 0.833 缩放 + "同一世界角"配对。
            //  上板结果：中间相机区域正常、两侧区域全错 ⇒ 180° 假设与实测不符。
            //
            //  反过来，**旧版恒等映射（源 0..1023 ↔ 全景 1024 列，重叠 256 px）
            //  一直是"大体正确、只有接缝重影"** —— 这是唯一贯穿全程的实测事实。
            //  恒等映射成立 ⇔ 每颗相机 1280 px ≈ 150°：
            //    · 1024 px = 120° ✓ 与 120° 间距一致
            //    · 重叠 30° = 256 px ✓ 与原常量一致
            //    · 源图密度 = 全景密度（8.53 px/度）⇒ 恒等映射即正确的角度重采样
            //  ⇒ 镜头真实 FOV ≈ 150°（用户给的 120/130/180 均不准），原始几何本来就对。
            //
            //  结论：几何恢复原样；接缝重影是"配准"问题（装配/镜头个体差异），
            //        不是投影模型问题，后续用每相机水平微调 b 解决，不再动全局映射。
            //-----------------------------------------------------------------

            is_blending_region_s0 = 1'b0; // 默认值
            if (pano_coord_s0 < OVERLAP_WIDTH) begin
                // 回绕缝融合（U5→U4 相接处）：cam3 尾列线性淡出、cam2 头列淡入。
                // ★ [v6 修复] 原 alpha_s0 = (pano_coord_s0 << 8)/OVERLAP_WIDTH 是坏的：
                //   pano_coord_s0 是 13bit，"<<"结果宽度=左操作数宽度 ⇒ <<8 后被截到 13bit，
                //   /256 后得到的是 pano mod 32（0..31 循环）⇒ 回绕缝 224/256 区域权重全错。
                //   （另两处缝因减法与 32bit localparam 运算把上下文扩宽而幸免。）
                //   (x<<8)/256 ≡ x，直接取低 8 位即可。
                is_blending_region_s0 = 1'b1;
                ix_a_s0     = pano_coord_s0[10:0] + 11'd1024;    // cam3 尾（1024..1279）
                ix_b_s0     = pano_coord_s0[10:0];               // cam2 头（0..255）
                data_left_s0  = cam3_line_buffer[src_a_w];
                data_right_s0 = cam2_line_buffer[src_b_w];
                alpha_s0 = pano_coord_s0[7:0];
            end
            else if (pano_coord_s0 < BLEND1_START) begin
                ix_a_s0 = pano_coord_s0[10:0];
                direct_data_s1 <= cam2_line_buffer[src_a_w];
            end
            else if (pano_coord_s0 < BLEND1_END) begin
                is_blending_region_s0 = 1'b1;
                ix_a_s0     = pano_coord_s0[10:0];               // cam2（1024..1279）
                ix_b_s0     = pano_coord_s0[10:0] - 11'd1024;    // cam1 头（0..255）
                data_left_s0  = cam2_line_buffer[src_a_w];
                data_right_s0 = cam1_line_buffer[src_b_w];
                alpha_s0 = pano_coord_s0[7:0];                   // pano-1024 ∈ 0..255
            end
            else if (pano_coord_s0 < BLEND2_START) begin
                // pano∈[1280,2048)：cam1 直通，本地列 = pano−1024 ∈ [256,1024)
                ix_a_s0 = pano_coord_s0[10:0] - 11'd1024;
                direct_data_s1 <= cam1_line_buffer[src_a_w];
            end
            else if (pano_coord_s0 < BLEND2_END) begin
                // pano∈[2048,2304)：cam1 尾 ↔ cam3 头
                is_blending_region_s0 = 1'b1;
                ix_a_s0     = pano_coord_s0[10:0] + 11'd1024;    // cam1 尾（1024..1279）
                ix_b_s0     = pano_coord_s0[10:0];               // cam3 头（0..255）
                data_left_s0  = cam1_line_buffer[src_a_w];
                data_right_s0 = cam3_line_buffer[src_b_w];
                alpha_s0 = pano_coord_s0[7:0];                   // pano-2048 ∈ 0..255
            end
            else if (pano_coord_s0 < PANORAMA_WIDTH) begin
                // pano∈[2304,3072)：cam3 直通，本地列 = pano−2048 ∈ [256,1024)
                ix_a_s0 = pano_coord_s0[10:0];
                direct_data_s1 <= cam3_line_buffer[src_a_w];
            end
            else begin
                direct_data_s1 <= 16'hF800; // 如果出错，显示红色用于debug
            end
        end
    end

    //=========================================================================
    //  [2026-11-02 新增] 人脸检测 + DOA 联动转向
    //=========================================================================

    // ---- 1) 视频域人脸检测：直接吃显示侧像素流 ----
    //   输出块坐标（0..159 / 0..119），×8 / ×6 即屏幕像素坐标。
    //   阈值已与 MIN_H 的宽高比自洽（≈3 m 可检），并用 MAX_W 拒绝大面积肤色背景。
    face_det_stream #(
        .BW (8), .BWC (3), .BH (6), .BHC (5), .CW (160), .CH (120),
        .BLK_THR (6), .ROW_THR (8),
        .MIN_H (8), .MAX_H (70), .MAX_W (128),
        .EYE_RATIO_Q8 (205)
    ) u_face_det (
        .clk_v      (video_clk),
        .rst_n      (rst_n),
        .px_de      (off0_syn_de),
        .px_data    (off0_syn_data[15:0]),
        .vs         (syn_off0_vs),
        .face_cx    (face_cx),
        .face_cy    (face_cy),
        .face_w     (face_w),
        .face_h     (face_h),
        .face_conf  (face_conf),
        .face_tgl   (face_tgl)
    );

    // ---- 2) DOA 主控转向 + 人脸微调居中 ----
    //   ★ 上板要调的都是这里的常量：
    //     SECT_OFF  —— 标定：正前方说话时 DOA 落在哪个扇区（0..11，每个 30°）
    //     SECT_DIR  —— 标定：扇区递增方向与全景列递增方向是否一致
    //     STEP_AIM  —— 切屏速度（24 px/帧 ≈ 360 px/s，切一颗相机 1024 px ≈ 2.8 s）
    //     STEP_FACE —— 人脸微调上限（8 px/帧 ≈ 120 px/s）
    //     DEAD_PX   —— 居中死区（16 px ≈ 2°），太小会抖、太大会偏
    pan_ctrl #(
        .PAN_W        (3072),
        .TGT_U4       (2944),          // cam2 段中心 512 的取景窗左边界
        .TGT_U5       (1920),          // cam3 段中心 2560
        .TGT_U3       (896),           // cam1 段中心 1536
        .STEP_AIM     (48),            // ★ 3 秒硬约束：15fps 下对面 1536px 需 ≥48px/帧
                                       //   （实测 40px/帧 要 3.33s；48px/帧 2.93s ✓）
                                       //   相邻相机 1024px 则约 2.9s
        .STEP_FACE    (4),             // ★ 人脸微调每帧上限 4px（60 px/s，"缓慢平移"）
        .DEAD_PX      (16),            // 居中死区 ≈2°
        .FACE_RANGE   (350),           // 微调允许偏离相机中心 ±41°
        .CONF_MIN     (32),            // ★ 无音源不得误触发：16→32
                                       //   噪声/环境声的 confidence 通常 <24，
                                       //   门限 32 可避免"没人在说话画面却乱转"；
                                       //   若真人说话也进不来，降到 24
        .FACE_CONF_MIN(16),
        .HOLD_N       (3),             // 换相机需连续 3 次确认
        .DWELL_N      (15),            // ★ 换相机后至少驻留 1 s 才允许再换
        .ACT_TICK     (15),            // 有声保持 ≈1 s
        .LOCK_TICK    (15),            // ★ 按键结束后 1 s 内禁止自动（需求②）
        // ★ 组 → 摄像头映射（0=U4 1=U5 2=U3），按硬件方向：
        //     M1/M0 ↔ U5 、 M5/M4 ↔ U4 、 M3/M2 ↔ U3
        .GRP0_CAM     (1),             // 组0 → U5
        .GRP1_CAM     (0),             // 组1 → U4
        .GRP2_CAM     (2),             // 组2 → U3
        .SECT_OFF     (4'd0),          // ★ 标定：正前方对应的扇区（每个 30°）
        .SECT_DIR     (1'b0)           // ★ 标定：扇区递增与全景列递增是否同向
    ) u_pan_ctrl (
        .clk          (clk),
        .rst_n        (rst_n),
        .step_tick    (step_tick),
        .doa_bearing  (doa_bearing),
        .doa_sector   (doa_sector),
        .doa_conf     (doa_conf),
        .doa_valid    (doa_valid),
        .fc_x         (face_cx),
        .fc_conf      (face_conf),
        .fc_tgl       (face_tgl),
        .any_key      (any_key),
        .manual_offset(manual_offset[12:0]),
        .auto_offset  (auto_offset),
        .auto_active  (auto_active),
        .dbg_cam      (dbg_cam),
        .dbg_sect     (dbg_sect)
    );

    //-------------------------------------------------------------------------
    // [2026-11-02 诊断] 把 DOA 联动链路的三个关键状态引到板载 LED
    //
    //   state_led[3] = DOA 在解算（doa_valid 脉冲展宽到 200 ms，会闪）
    //   state_led[2] = DOA 置信度达标（conf >= 16，持续亮）
    //   state_led[1] = 自动接管中（auto_active，亮 = 正在自动转向）
    //
    //   判读表（下次"不转"时直接看这三个灯就能定位）：
    //     全灭            → DOA 从未解算：麦克风没接/没信号/时钟不对
    //     [3]闪 [2]灭     → 有解算但置信度不够：调小 CONF_MIN 或查麦克风增益
    //     [2]亮 [1]灭     → 逻辑层问题：按键被误判为按下（any_key 恒 1）
    //     [1]亮但画面不动 → pan_ctrl→offset 通路问题（看 auto_offset 是否在变）
    //-------------------------------------------------------------------------
    reg [23:0] doa_blink;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                 doa_blink <= 24'd0;
        else if (doa_valid)         doa_blink <= 24'd10_000_000;   // ≈200 ms @50 MHz
        else if (doa_blink != 24'd0) doa_blink <= doa_blink - 24'd1;
    end

    wire doa_ok_top = doa_valid && (doa_conf >= 8'd16);

    assign state_led[3] = (doa_blink != 24'd0);
    assign state_led[2] = doa_ok_top;
    assign state_led[1] = auto_active;

end
endgenerate

    
    Video_Frame_Buffer_Top Video_Frame_Buffer_Top_inst
    ( 
        .I_rst_n              (init_calib_complete ),
        .I_dma_clk            (dma_clk          ),
    `ifdef USE_THREE_FRAME_BUFFER 
        .I_wr_halt            (1'd0             ), //1:halt,  0:no halt
        .I_rd_halt            (1'd0             ), //1:halt,  0:no halt
    `endif

        // video data input       
        .I_vin0_clk           (fb_vin_clk   ),
        .I_vin0_vs_n          (~fb_vin_vsync),//只接收负极性
        .I_vin0_de            (fb_vin_de    ),
        .I_vin0_data          (fb_vin_data  ),
        .O_vin0_fifo_full     (             ),

        // video data output            
        .I_vout0_clk          (video_clk        ),
        .I_vout0_vs_n         (~syn_off0_vs     ),//只接收负极性
        .I_vout0_de           (out_de           ),
        .O_vout0_den          (off0_syn_de      ),
        .O_vout0_data         (off0_syn_data    ),
        .O_vout0_fifo_empty   (                 ),
        // ddr write request
        .I_cmd_ready          (cmd_ready          ),
        .O_cmd                (cmd                ),//0:write;  1:read
        .O_cmd_en             (cmd_en             ),
    //    .O_app_burst_number   (app_burst_number   ),
        .O_addr               (addr               ),//[ADDR_WIDTH-1:0]
        .I_wr_data_rdy        (wr_data_rdy        ),
        .O_wr_data_en         (wr_data_en         ),//
        .O_wr_data_end        (wr_data_end        ),//
        .O_wr_data            (wr_data            ),//[DATA_WIDTH-1:0]
        .O_wr_data_mask       (wr_data_mask       ),
        .I_rd_data_valid      (rd_data_valid      ),
        .I_rd_data_end        (rd_data_end        ),//unused 
        .I_rd_data            (rd_data            ),//[DATA_WIDTH-1:0]
        .I_init_calib_complete(init_calib_complete)
    ); 

    reg [1:0] DDRPLL_LOCK_R;
    wire DDRPLL_STOP;
    reg DDRPLL_STOP_R;
    reg DDRPLL_WR;

    always@(posedge clk)
    begin
        DDRPLL_LOCK_R[0] <= DDR_pll_lock;
        DDRPLL_LOCK_R[1] <= DDRPLL_LOCK_R[0];

        DDRPLL_STOP_R <= DDRPLL_STOP;

        if(!DDRPLL_LOCK_R[1])
            DDRPLL_WR <= 1'b0;
        else if(~DDRPLL_STOP & DDRPLL_STOP_R) begin
            DDRPLL_WR <= 1'b1;
        end else if(DDRPLL_STOP & (~DDRPLL_STOP_R))begin
            DDRPLL_WR <= 1'b1;
        end else begin
            DDRPLL_WR <= 1'b0;
        end
    end
    // for 25K & 60K, not for 138K
    pll_mDRP_intf DDRPLL_STOP_inst(
        .clk(clk),
        .rst_n(rst_n),
        .pll_lock(DDRPLL_LOCK_R[1]),
        .wr(DDRPLL_WR),
        .mdrp_inc(DDRPLL_MDR_AINC),
        .mdrp_op(DDRPLL_MDR_OPC),
        .mdrp_wdata(DDRPLL_MDR_DI),
        .mdrp_rdata(DDRPLL_MDR_DO)
    );    

    DDR3MI u_ddr3
    (
        .clk                (clk                ),
        .memory_clk         (memory_clk         ),
        .pll_stop           (DDRPLL_STOP        ),
        .pll_lock           (DDR_pll_lock       ),
        .rst_n              (rst_n              ),
        //    .app_burst_number   (app_burst_number   ),
        .cmd_ready          (cmd_ready          ),
        .cmd                (cmd                ),
        .cmd_en             (cmd_en             ),
        .addr               (addr               ),
        .wr_data_rdy        (wr_data_rdy        ),
        .wr_data            (wr_data            ),
        .wr_data_en         (wr_data_en         ),
        .wr_data_end        (wr_data_end        ),
        .wr_data_mask       (wr_data_mask       ),
        .rd_data            (rd_data            ),
        .rd_data_valid      (rd_data_valid      ),
        .rd_data_end        (rd_data_end        ),
        .sr_req             (1'b0               ),
        .ref_req            (1'b0               ),
        .sr_ack             (                   ),
        .ref_ack            (                   ),
        .init_calib_complete(init_calib_complete),
        .clk_out            (dma_clk            ),
        .burst              (1'b1               ),
        // mem interface
        .ddr_rst            (                 ),
        .O_ddr_addr         (ddr_addr         ),
        .O_ddr_ba           (ddr_bank         ),
        .O_ddr_cs_n         (ddr_cs           ),
        .O_ddr_ras_n        (ddr_ras          ),
        .O_ddr_cas_n        (ddr_cas          ),
        .O_ddr_we_n         (ddr_we           ),
        .O_ddr_clk          (ddr_ck           ),
        .O_ddr_clk_n        (ddr_ck_n         ),
        .O_ddr_cke          (ddr_cke          ),
        .O_ddr_odt          (ddr_odt          ),
        .O_ddr_reset_n      (ddr_reset_n      ),
        .O_ddr_dqm          (ddr_dm           ),
        .IO_ddr_dq          (ddr_dq           ),
        .IO_ddr_dqs         (ddr_dqs          ),
        .IO_ddr_dqs_n       (ddr_dqs_n        )
    );


    // DDR Output video Timing Align
    //---------------------------------------------
    wire [4:0] lcd_r,lcd_b;
    wire [5:0] lcd_g;
    wire lcd_vs,lcd_de,lcd_hs,lcd_dclk;
    
    assign {lcd_r,lcd_g,lcd_b}    = off0_syn_de ? off0_syn_data[15:0] : 16'h0000;//{r,g,b}
    assign lcd_vs      			  = Pout_vs_dn[1];//syn_off0_vs;
    assign lcd_hs      			  = Pout_hs_dn[1];//syn_off0_hs;
    assign lcd_de      			  = Pout_de_dn[1];//off0_syn_de;
    assign lcd_dclk    			  = video_clk;//video_clk_phs;

    reg  [1:0]  Pout_hs_dn;
    reg  [1:0]  Pout_vs_dn;
    reg  [1:0]  Pout_de_dn;

    always@(posedge video_clk or negedge rst_n)
    begin
        if(!rst_n)
            begin                          
                Pout_hs_dn  <= {2'b11};
                Pout_vs_dn  <= {2'b11}; 
                Pout_de_dn  <= {2'b00}; 
            end
        else 
            begin                          
                Pout_hs_dn  <= {Pout_hs_dn[0],syn_off0_hs};
                Pout_vs_dn  <= {Pout_vs_dn[0],syn_off0_vs}; 
                Pout_de_dn  <= {Pout_de_dn[0],out_de}; 
            end
    end

    //==============================================================================
    //TMDS TX(HDMI4)
    wire serial_clk;
    wire hdmi4_rst_n;

    TMDS_PLL u_tmds_pll(
        .clkin     (clk              ),     //input clk 
        .clkout0   (serial_clk       ),     //output clk x5ni
        .clkout1   (video_clk        ),     //output clk x1
        .lock      (TMDS_DDR_pll_lock)      //output lock
        );

    assign hdmi4_rst_n = rst_n & TMDS_DDR_pll_lock;

    wire dvi0_rgb_clk;
    wire dvi0_rgb_vs ;
    wire dvi0_rgb_hs ;
    wire dvi0_rgb_de ;
    wire [7:0] dvi0_rgb_r  ;
    wire [7:0] dvi0_rgb_g  ;
    wire [7:0] dvi0_rgb_b  ;

    wire dvi1_rgb_clk;
    wire dvi1_rgb_vs ;
    wire dvi1_rgb_hs ;
    wire dvi1_rgb_de ;
    wire [7:0] dvi1_rgb_r  ;
    wire [7:0] dvi1_rgb_g  ;
    wire [7:0] dvi1_rgb_b  ;

generate if(DDR_BYPASS == "true")
begin
    //DVI directly use TPG video
    assign dvi0_rgb_clk = video_clk ;
    assign dvi0_rgb_vs  = tp0_vs_in ;
    assign dvi0_rgb_hs  = tp0_hs_in ;
    assign dvi0_rgb_de  = tp0_de_in ;
    assign dvi0_rgb_r   = tp0_data_r;
    assign dvi0_rgb_g   = tp0_data_g;
    assign dvi0_rgb_b   = tp0_data_b;
end else begin
    assign dvi0_rgb_clk = lcd_dclk;
    assign dvi0_rgb_vs  = lcd_vs;
    assign dvi0_rgb_hs  = lcd_hs;
    assign dvi0_rgb_de  = lcd_de;
    assign dvi0_rgb_r   = {lcd_r,3'd0};
    assign dvi0_rgb_g   = {lcd_g,2'd0};
    assign dvi0_rgb_b   = {lcd_b,3'd0};
end
endgenerate

    DVI_TX_Top DVI_TX_Top_inst0
    (
        .I_rst_n       (hdmi4_rst_n   ),  //asynchronous reset, low active
        .I_serial_clk  (serial_clk    ),

        //CMOS
        .I_rgb_clk     (dvi0_rgb_clk),  //pixel clock
        .I_rgb_vs      (dvi0_rgb_vs ), 
        .I_rgb_hs      (dvi0_rgb_hs ),    
        .I_rgb_de      (dvi0_rgb_de ), 
        .I_rgb_r       (dvi0_rgb_r  ), 
        .I_rgb_g       (dvi0_rgb_g  ),  
        .I_rgb_b       (dvi0_rgb_b  ),  

        .O_tmds_clk_p  (tmds_clk_p_0  ),
        .O_tmds_clk_n  (tmds_clk_n_0  ),
        .O_tmds_data_p (tmds_d_p_0    ),  //{r,g,b}
        .O_tmds_data_n (tmds_d_n_0    )
    );

    //=========================================================================
    //  诊断观测器：三路帧号 / FIFO 溢出 / 行错位 / I2C 配置轮次 → HSPI → PC
    //  （docs/TRIPLE_CAM_PERF_PLAN.md 第 0 步；引脚经核实与既有 109 脚零冲突）
    //=========================================================================
    cam_diag u_cam_diag (
        .clk               (clk),
        .rst_n             (rst_n),
        .cam1_clk          (cmos_16bit_clk),
        .cam1_vs           (cmos_vsync),
        .cam2_vs_pulse     (cam2_vsync_synced_pulse),
        .cam3_vs_pulse     (cam3_vsync_synced_pulse),
        .cam2_fifo_full    (cam2_fifo_full),
        .cam3_fifo_full    (cam3_fifo_full),
        .cam2_wr_ptr       (line_buffer_wr_ptr_cam2),
        .cam3_wr_ptr       (line_buffer_wr_ptr_cam3),
        .pixel_counter     (pixel_counter),
        .pixel_en          (cmos_16bit_wr),
        .lut_index_cam1    (lut_index_cam1),
        .lut_index_cam2    (lut_index_cam2),
        .lut_index_cam3    (lut_index_cam3),
        .i2c_err_cam1      (i2c_err_cam1),
        .i2c_err_cam2      (i2c_err_cam2),
        .i2c_err_cam3      (i2c_err_cam3),
        .horizontal_offset (horizontal_offset),
        .hspi_htclk        (hspi_htclk),
        .hspi_htreq        (hspi_htreq),
        .hspi_htrdy        (hspi_htrdy),
        .hspi_htvld        (hspi_htvld),
        .hspi_hd           (hspi_hd),
        .diag_uart_tx      (diag_uart_tx),
        .diag_uart_tx2     (uart_vid_tx2)
    );

    //=========================================================================
    //  AUDIO SUBSYSTEM  ——  7 麦阵列采集 + TDOA/DOA 解算 + SK9822 灯环
    //
    //  与视频链路完全解耦：音频子系统只用共享的 clk(50MHz)/rst_n，
    //  内部自行用 clk_div(clk/16) 产生 I2S 主时钟 BCK，不占用任何 PLL，
    //  也不与视频侧共享寄存器或状态。两边唯一的数据交汇点是 UART 合流器。
    //
    //  引脚：8 根飞线全部落在原 SDRAM BANK 的 3.3V 脚
    //        (A13/A14/B13/B15/B16/C13/C14/C15)，视频侧完全不用这些脚，
    //        因此与三路摄像头 + DDR3 + HDMI 无任何冲突。
    //=========================================================================
    audio_top #(
        .UART_PKT_MODE (2)      // 2 = DOA 文本诊断(ASCII)，115200 8N1 直接可读
    ) u_audio (
        .clk            (clk),
        .rst_n          (rst_n),
        .mic_array_bck  (mic_array_bck),
        .mic_array_ws   (mic_array_ws),
        .mic_array_d    (mic_array_d),
        .mic_led_clk    (mic_led_clk),
        .mic_led_dat    (mic_led_dat),
        .uart_txp       (uart_aud_tx),
        .leds           (mic_leds),
        // [新增] DOA 结果 → pan_ctrl（自动转向的主控输入）
        .doa_bearing_o  (doa_bearing),
        .doa_sector_o   (doa_sector),
        .doa_conf_o     (doa_conf),
        .doa_valid_o    (doa_valid)
    );

    //=========================================================================
    //  UART 合流：把「视频诊断行」和「麦克风诊断行」并到同一个 U15 脚
    //
    //  两路都是 115200 8N1 的独立发送器，直接线或会产生乱码，所以这里做
    //  字节级互斥：谁先发出起始位谁占用线路，发满 10 bit 才释放；
    //  同一时刻都有请求时视频优先。被让路的一方丢一个字节（不是半字节），
    //  因此接收端永远看不到 framing error，最坏情况只是文本里缺一个字符。
    //=========================================================================
    uart_merge u_uart_merge (
        .clk        (clk),
        .rst_n      (rst_n),
        .a_tx       (uart_vid_tx2),   // 视频诊断（高优先级）
        .b_tx       (uart_aud_tx),    // 麦克风诊断
        .tx_out     (uart_pc_tx),
        .conflict   (uart_conflict)
    );



endmodule