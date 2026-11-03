//-----------------------------------------------------------------------------
// subband4.v
//   A four-band analysis filterbank, ONE first-order one-pole tree, for the
//   sub-band PHAT correlator (see tools/golden_subband_phat.py for the model
//   that fixed its band edges and coefficients).
//
//   The tree is COMPLEMENTARY:  b0 + b1 + b2 + b3 == x  exactly, so the four
//   bands re-sum to the input (which is why the broadband energy can still be
//   taken from the input, not reconstructed).
//
//       lo1 = LP(x,    a=1/4)         fc ~ 2252 Hz
//       hi1 = x - lo1                 HP @ 2252
//       b0  = LP(lo1,  a=1/16)        fc ~ 486 Hz    -> [0,    486]
//       b1  = lo1 - b0                                -> [486,  2252]
//       b2  = LP(hi1,  a=1/2)         fc ~ 5616 Hz    -> [2252, 5616]
//       b3  = hi1 - b2                                -> [5616, +]
//
//   (The top band is capped in practice by mic_cond's 6 kHz low-pass upstream.)
//
//   Every coefficient is a power of two, so each section is ONE subtract, ONE
//   arithmetic shift and ONE add -- no multiplier, no coefficient RAM, nothing
//   to tune at run time.  This matches the multiplier-free style of mic_cond.
//
//   WHY ONE-POLE SECTIONS AND NOT A BRICK-WALL BANK
//   -----------------------------------------------
//   The golden model measured BOTH: a 4-band one-pole tree OUTPERFORMS a
//   brick-wall (FFT) split at the same edges (82% vs 25% hit rate on the
//   interferer case).  A brick-wall bank has a sinc impulse response -- long
//   time-domain ringing -- and splits each band so narrowly that its
//   correlation peak oscillates.  Soft one-pole edges keep each band broad
//   enough to hold a sharp, unambiguous correlation peak, which is the whole
//   point of the exercise.
//
//   Widths follow mic_cond: IW = DATA_W + 2 guard bits, output clamped to
//   DATA_W.  The clamp is a real guard (the complementary HP can momentarily
//   overshoot), not cold-start belt-and-braces.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module subband4 #(
    parameter integer DATA_W = 24
) (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     en,       // one cycle per input sample
    input  wire signed [DATA_W-1:0] x,
    output reg  signed [DATA_W-1:0] b0,
    output reg  signed [DATA_W-1:0] b1,
    output reg  signed [DATA_W-1:0] b2,
    output reg  signed [DATA_W-1:0] b3
);
    localparam integer IW = DATA_W + 2;

    // sign-extend the input to the guard width
    wire signed [IW-1:0] xg = {{2{x[DATA_W-1]}}, x};

    // ---- section state (previous outputs of the three LP stages) ----
    reg signed [IW-1:0] s_lo1;
    reg signed [IW-1:0] s_b0;
    reg signed [IW-1:0] s_b2;

    // ---- one-pole LP:  y_next = s + ((x - s) >>> k),  a = 2^-k ----
    // (computed combinationally from the CURRENT state; all three sections
    //  update on the same `en` edge, exactly like mic_cond)
    wire signed [IW-1:0] lo1 = s_lo1 + ((xg    - s_lo1) >>> 2);   // a=1/4
    wire signed [IW-1:0] hi1 = xg - lo1;
    wire signed [IW-1:0] b0w = s_b0  + ((lo1   - s_b0 ) >>> 4);   // a=1/16
    wire signed [IW-1:0] b2w = s_b2  + ((hi1   - s_b2 ) >>> 1);   // a=1/2

    // ---- output clamp to DATA_W ----
    localparam signed [IW-1:0] Y_MAX =  (1 <<< (DATA_W-1)) - 1;
    localparam signed [IW-1:0] Y_MIN = -(1 <<< (DATA_W-1));

    function signed [DATA_W-1:0] clp;
        input signed [IW-1:0] v;
        reg   signed [IW-1:0] t;
        begin
            t = (v > Y_MAX) ? Y_MAX : (v < Y_MIN) ? Y_MIN : v;
            clp = t[DATA_W-1:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_lo1 <= {IW{1'b0}};
            s_b0  <= {IW{1'b0}};
            s_b2  <= {IW{1'b0}};
            b0    <= {DATA_W{1'b0}};
            b1    <= {DATA_W{1'b0}};
            b2    <= {DATA_W{1'b0}};
            b3    <= {DATA_W{1'b0}};
        end else if (en) begin
            s_lo1 <= lo1;
            s_b0  <= b0w;
            s_b2  <= b2w;
            b0    <= clp(b0w);
            b1    <= clp(lo1 - b0w);
            b2    <= clp(b2w);
            b3    <= clp(hi1 - b2w);
        end
    end

endmodule
