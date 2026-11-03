//-----------------------------------------------------------------------------
// i2s_rx_8slot.v
//   Sipeed 6+1 Mic Array I2S receiver for Tang MEGA 60K (GW5AT-LV60PG484A)
//
//   Hardware (from D:/Gowin/tang_mega_60K_pins.cst):
//     mic_array_bck  -> Y11   bit clock           (LVCMOS15, BANK_VCCIO=1.5)
//     mic_array_ws   -> W10   word select / LRCLK (LVCMOS15, BANK_VCCIO=1.5)
//     mic_array_d[3] -> AB11  data lane 3         (LVCMOS15)
//     mic_array_d[2] -> AA11  data lane 2         (LVCMOS15)
//     mic_array_d[1] -> AB10  data lane 1         (LVCMOS15)
//     mic_array_d[0] -> AA10  data lane 0         (LVCMOS15)
//
//   FRAME STRUCTURE
//   ---------------
//   One frame = one WS-low half + one WS-high half.
//   In each half the 4 data lanes present their own DATA_W-bit word in
//   parallel, MSB first, starting one bck after the WS edge.
//
//   Words are named by (lane, phase) and appear as:
//       slot0 = lane0 ws-low    slot1 = lane0 ws-high
//       slot2 = lane1 ws-low    slot3 = lane1 ws-high
//       slot4 = lane2 ws-low    slot5 = lane2 ws-high
//       slot6 = lane3 ws-low    slot7 = lane3 ws-high
//
//   LOCKING
//   -------
//   The array and the FPGA share no reset, so a free-running bit counter
//   drifts.  Every WS transition therefore RESETS the half-frame counter.
//   The receiver is self-aligning and re-locks on each WS edge.
//
//   BIT ALIGNMENT
//   -------------
//   Inputs are registered once (ws_r, d_r) to give the router clean input
//   flops.  This costs exactly 1 bck.  The source presents the MSB 1 bck
//   after the WS edge, so on d_r the MSB of a half is present when the
//   half's counter is 0.  We shift on bit_cnt = 0 .. DATA_W-1.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module i2s_rx_8slot #(
    parameter integer DATA_W = 24,      // I2S data bits per word
    parameter integer SLOT_W = 32       // bck cycles per half-frame
) (
    input  wire                     rst_n,
    input  wire                     bck,        // bit clock (from mic array)
    input  wire                     ws,         // word select (from mic array)
    input  wire [3:0]               d_in,       // 4 data lanes

    output reg  signed [DATA_W-1:0] slot0,      // lane0 WS-low
    output reg  signed [DATA_W-1:0] slot1,      // lane0 WS-high
    output reg  signed [DATA_W-1:0] slot2,      // lane1 WS-low
    output reg  signed [DATA_W-1:0] slot3,      // lane1 WS-high
    output reg  signed [DATA_W-1:0] slot4,      // lane2 WS-low
    output reg  signed [DATA_W-1:0] slot5,      // lane2 WS-high
    output reg  signed [DATA_W-1:0] slot6,      // lane3 WS-low
    output reg  signed [DATA_W-1:0] slot7,      // lane3 WS-high

    output reg                      frame_stb,  // 1 bck wide, end of frame
    output reg                      ws_phase,   // WS level of the current half
    output reg                      locked      // 1 once framing is acquired
);

    // CNT_W is derived from SLOT_W so the parameter is load-bearing rather
    // than decorative: it used to be declared and never referenced, so
    // editing it changed nothing at all.
    localparam integer CNT_W = (SLOT_W < 2) ? 1 : $clog2(SLOT_W);

    //------------------------------------------------------------------
    // Input registers (single stage, see header note on bit alignment).
    //------------------------------------------------------------------
    reg        ws_r;
    reg [3:0]  d_r;

    always @(posedge bck or negedge rst_n) begin
        if (!rst_n) begin
            ws_r <= 1'b0;
            d_r  <= 4'h0;
        end else begin
            ws_r <= ws;
            d_r  <= d_in;
        end
    end

    //------------------------------------------------------------------
    // WS edge detect (registered, so glitch-free in the bck domain).
    //------------------------------------------------------------------
    reg        ws_r_d;
    wire       ws_edge = (ws_r != ws_r_d);

    //------------------------------------------------------------------
    // Half-frame bit counter and the 4 parallel lane accumulators.
    //------------------------------------------------------------------
    reg [CNT_W-1:0]   bit_cnt;
    reg               half;         // WS level of the half being received
    reg [DATA_W-1:0]  sh0, sh1, sh2, sh3;

    wire              shifting  = (bit_cnt < DATA_W);

    always @(posedge bck or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt   <= { CNT_W {1'b0} };
            ws_r_d    <= 1'b0;
            half      <= 1'b0;
            sh0 <= { DATA_W {1'b0} };
            sh1 <= { DATA_W {1'b0} };
            sh2 <= { DATA_W {1'b0} };
            sh3 <= { DATA_W {1'b0} };
            frame_stb <= 1'b0;
            ws_phase  <= 1'b0;
            locked    <= 1'b0;
            slot0 <= 0; slot1 <= 0; slot2 <= 0; slot3 <= 0;
            slot4 <= 0; slot5 <= 0; slot6 <= 0; slot7 <= 0;
        end else begin
            ws_r_d    <= ws_r;
            frame_stb <= 1'b0;

            if (ws_edge) begin
                // A new half-frame starts.  Restart the counter and clear
                // the accumulators so the 4 lane words are captured cleanly.
                bit_cnt <= { CNT_W {1'b0} };
                half    <= ws_r;
                ws_phase<= ws_r;
                sh0 <= { DATA_W {1'b0} };
                sh1 <= { DATA_W {1'b0} };
                sh2 <= { DATA_W {1'b0} };
                sh3 <= { DATA_W {1'b0} };

                if (ws_r == 1'b0) begin
                    // WS fell -> a low half just completed -> frame boundary.
                    // The very first low half after reset has no valid
                    // preceding state, so it merely ACQUIRES lock; from the
                    // second frame onwards the data is valid.
                    frame_stb <= locked;
                    if (!locked) locked <= 1'b1;
                end
            end else begin
                // The receiver is self-aligning -- every WS edge resets the
                // counter -- so SLOT_W is not needed to FIND the frame
                // boundary.  It does size and clamp the counter, which is
                // what makes the parameter meaningful: a half-frame longer
                // than SLOT_W now stalls the counter instead of silently
                // wrapping past the data window.
                if (bit_cnt == SLOT_W-1)
                    bit_cnt <= bit_cnt;
                else
                    bit_cnt <= bit_cnt + 1'b1;

                // shift all four lanes MSB-first while inside the window
                if (shifting) begin
                    sh0 <= { sh0[DATA_W-2:0], d_r[0] };
                    sh1 <= { sh1[DATA_W-2:0], d_r[1] };
                    sh2 <= { sh2[DATA_W-2:0], d_r[2] };
                    sh3 <= { sh3[DATA_W-2:0], d_r[3] };
                end

                // on the bck AFTER the last bit was shifted, the
                // accumulators hold the complete word -> commit it.
                if (bit_cnt == DATA_W) begin
                    if (half == 1'b0) begin
                        slot0 <= sh0; slot2 <= sh1;
                        slot4 <= sh2; slot6 <= sh3;
                    end else begin
                        slot1 <= sh0; slot3 <= sh1;
                        slot5 <= sh2; slot7 <= sh3;
                    end
                end
            end
        end
    end

endmodule
