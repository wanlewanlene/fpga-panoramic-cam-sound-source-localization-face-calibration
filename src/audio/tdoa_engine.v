//-----------------------------------------------------------------------------
// tdoa_engine.v
//   The TDOA front end, in two modules:
//     tdoa_engine -- ONE sub-band-PHAT cross-correlation datapath, shared in
//                    time across the six ring microphones, referenced to the
//                    centre mic;
//     tdoa_filter -- per-microphone low-pass on that engine's sub-sample
//                    result (end of this file).
//   They share one file because they share one contract: {lag, frac} with
//   tau = lag * 2^FRAC_W + frac.
//
//   SUB-BAND PHAT (2026-10-19)
//   --------------------------
//   The correlator is no longer a single broadband cross-correlation.  Each
//   channel is split into NBAND one-pole bands (mic_capture.v / subband4.v)
//   and the correlation is the ENERGY-NORMALISED SUM over bands:
//
//       R[k] = sum_b  C_b[k] >>> s_b          (then rescaled by << NORM_G)
//       C_b[k] = sum_{n} (x_b[n+k] >>> SHIFT) * (r_b[n] >>> SHIFT)
//       s_b    = max(0, floor(log2(sqrt(E_x[b] * E_r[b]))) - NORM_KEEP)
//
//   The per-band energy shift is a coarse PHAT whitening: a loud band (large
//   E) is shifted right MORE, so no band dominates the peak.  It is what
//   turns a common-mode low-frequency interferer -- the classic reason a
//   bearing drifts toward the array centre -- into a band that is silently
//   de-weighted.
//
//   WHY A SHIFT AND NOT A DIVISION / MULTIPLIER
//   -------------------------------------------
//   tools/golden_subband_phat.py measured BOTH: a shift normalisation hits
//   97/96/76 % correct-lag (interferer / light / heavy reverb) against
//   99/93/81 % for exact float division -- close enough, and the shift costs
//   a barrel shifter OUTSIDE the MAC critical path instead of a 40x40
//   multiplier.  A brick-wall (FFT) bank was WORSE (25/19/14 %): sinc ringing
//   and over-narrow bands.  Soft one-pole bands + energy shift is the best
//   point on the cost/robustness curve.
//
//   The single shared datapath, the sub-sample parabolic interpolation, the
//   restoring divider and the flat-port conventions are unchanged from the
//   broadband engine and are documented there.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tdoa_engine #(
    parameter integer DATA_W  = 24,   // PCM width at the input
    parameter integer STORE_W = 16,   // width kept in the history buffers
    parameter integer N       = 256,  // correlation window length
    parameter integer L       = 2,    // lag range: -L .. +L
    parameter integer SHIFT   = 4,    // right shift per operand before MAC
    parameter integer ACC_W   = 36,   // accumulator width
    parameter integer FRAC_W  = 10,   // fractional bits in the peak offset
    parameter integer NMIC    = 6,    // ring microphones
    parameter integer NBAND   = 4,    // sub-band count

    //  PEAK_ABS -- how the correlation peak is CHOSEN, and the single most
    //  important knob for "loudness needed before the lamps react".
    //
    //  The peak search used to compare the SIGNED correlation R against the
    //  running best, so it only ever accepted a POSITIVE peak, and
    //  `valid_flat <= (best > 0)` threw away every microphone whose peak
    //  happened to be negative.  A negative correlation peak at the true lag
    //  simply means that channel is POLARITY INVERTED relative to the
    //  reference (x_m = -r[n-tau] -> C[tau] = -sum r^2): the DELAY IS STILL
    //  CORRECT, only the sign of the peak is flipped.  Inverting a channel
    //  costs nothing acoustically, and it happens whenever one data slot has
    //  its WS edge on the opposite phase or a MEMS part is mounted reversed.
    //
    //  With the old rule such a channel produced a lag taken from wherever
    //  the largest POSITIVE lobe happened to sit -- usually half a period
    //  away.  That is not a weak estimate, it is a WRONG one, so the robust
    //  fit had to reject it and the inlier count collapsed.  On the bench the
    //  field log showed V=09: only mic0 and mic3 valid, every other channel
    //  inverted.  2 inliers cap the quality at 255*2/6 = 85 and, with any
    //  residual at all, it sits under CONF_MIN -- so the ring stayed dark
    //  until the talker shouted loud enough to swamp the wrong lobe.
    //
    //  PEAK_ABS = 1 matches on |C| and marks a channel valid whenever any
    //  peak was found.  Polarity no longer matters, all six channels stay in
    //  the fit, and quality recovers without loosening a single threshold --
    //  which is why noise rejection does NOT get worse.  Set it to 0 to
    //  restore the legacy behaviour bit-for-bit.
    parameter integer PEAK_ABS = 1
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // ---- sample streaming ----------------------------------------------
    input  wire                     valid,
    input  wire signed [DATA_W-1:0] ref_full,               // broadband ref (energy gate)
    input  wire signed [NBAND*DATA_W-1:0] ref_b,            // NBAND bands of the ref
    input  wire signed [NBAND*DATA_W*NMIC-1:0] mic_b,       // NBAND bands x NMIC mics

    // ---- results -------------------------------------------------------
    output reg                      done,
    output reg  [4*NMIC-1:0]        lag_flat,
    output reg  [(FRAC_W+1)*NMIC-1:0] frac_flat,
    output reg  [ACC_W*NMIC-1:0]    mag_flat,
    output reg  [NMIC-1:0]          valid_flat,
    output reg                      any_valid,
    output reg  signed [ACC_W-1:0]  ref_energy
);

    //==================================================================
    //  Derived constants
    //==================================================================
    localparam integer NLAG    = 2*L + 1;
    localparam integer AW_N    = $clog2(N);
    localparam integer AW_X    = $clog2(N + 2*L);
    localparam integer XDEPTH  = N + 2*L;
    localparam integer MICSEL_W = (NMIC <= 2) ? 1 : $clog2(NMIC);
    localparam integer BAND_W  = (NBAND <= 2) ? 1 : $clog2(NBAND);

    localparam integer NORM_KEEP = 16;  // sum-of-abs norm: log2(N)=8 offset + 8 bits kept
    localparam integer NORM_G    = 26;  // rescale the band sum to ~2^36
    localparam [6:0]   NORM_KEEP_W = NORM_KEEP;   // 7-bit, matches norm_shift's s

    localparam integer SHIFT_REF = 4;
    localparam integer REF_SH    = (SHIFT <= SHIFT_REF) ? 2*(SHIFT_REF - SHIFT) : 0;

    localparam [AW_N-1:0] N_LAST = N - 1;
    localparam [AW_X-1:0] XDEPTH_LAST = XDEPTH - 1;
    localparam [3:0]      L4 = L;

    //==================================================================
    //  Correlation FSM states (4 bits: 9 states)
    //==================================================================
    localparam [3:0] S_IDLE   = 4'd0,
                     S_ENERGY = 4'd1,
                     S_SHIFT  = 4'd2,
                     S_MAC    = 4'd3,
                     S_NEXT   = 4'd4,
                     S_LAG    = 4'd5,
                     S_INTERP = 4'd6,
                     S_DIV    = 4'd7,
                     S_EMIT   = 4'd8,
                     S_WARM   = 4'd9;

    //==================================================================
    //  Flat-port index helpers.
    //==================================================================
    function integer LAGIDX;
        input [MICSEL_W-1:0] m;
        begin LAGIDX = 4 * m; end
    endfunction

    function integer ACCIDX;
        input [MICSEL_W-1:0] m;
        begin ACCIDX = ACC_W * m; end
    endfunction

    function integer FRACIDX;
        input [MICSEL_W-1:0] m;
        begin FRACIDX = (FRAC_W+1) * m; end
    endfunction

    function integer XBASE;
        input [MICSEL_W-1:0] m;
        input [BAND_W-1:0]   b;
        begin XBASE = (m * NBAND + b) * XDEPTH; end
    endfunction

    function integer RBASE;
        input [BAND_W-1:0]   b;
        begin RBASE = b * XDEPTH; end
    endfunction

    function [3:0] LI2IDX;
        input signed [3:0] li;
        begin LI2IDX = li + L4; end
    endfunction

    //==================================================================
    //  Sample truncation to STORE_W bits.
    //==================================================================
    function signed [STORE_W-1:0] trunc_sat;
        input signed [DATA_W-1:0] v;
        reg   signed [DATA_W-1:0] t;
        begin
            if (DATA_W == STORE_W) begin
                trunc_sat = v[STORE_W-1:0];
            end else begin
                t = v >>> (DATA_W - STORE_W);
                if      (t >  ((1 <<< (STORE_W-1)) - 1))
                    trunc_sat =  (1 <<< (STORE_W-1)) - 1;
                else if (t < -(1 <<< (STORE_W-1)))
                    trunc_sat = -(1 <<< (STORE_W-1));
                else
                    trunc_sat = t[STORE_W-1:0];
            end
        end
    endfunction

    // floor(log2(v)): index of the highest set bit, 0 for v == 0.
    function [5:0] flog2;
        input [ACC_W-1:0] v;
        reg   [5:0] r;
        integer i;
        begin
            r = 6'd0;
            for (i = 0; i < ACC_W; i = i + 1)
                if (v[i]) r = i[5:0];
            flog2 = r;
        end
    endfunction

    // per-band normalisation shift (>= 0).
    function [5:0] norm_shift;
        input [ACC_W-1:0] ex;
        input [ACC_W-1:0] er;
        reg   [6:0] s;
        reg   [6:0] d;
        begin
            // flog2 is 6 bits; with sum-of-abs energy the two add directly
            // (no >>1): s = flog2(Σ|x|)+flog2(Σ|r|) in 0..42, so widen BEFORE
            // the add or a loud band wraps (32+32 -> 0) and is silently NOT
            // de-weighted -- found by tb_tdoa_engine's control case.
            s = {1'b0, flog2(ex)} + {1'b0, flog2(er)};
            if (s < NORM_KEEP_W)
                norm_shift = 6'd0;
            else begin
                d = s - NORM_KEEP_W;    // 0..31
                norm_shift = d[5:0];
            end
        end
    endfunction

    //==================================================================
    //  History buffers (written in S_IDLE, read by the FSM).
    //==================================================================
    reg signed [STORE_W-1:0] xmem  [0:NMIC*NBAND*XDEPTH-1];
    reg signed [STORE_W-1:0] rmem  [0:NBAND*XDEPTH-1];
    reg signed [STORE_W-1:0] rfull [0:XDEPTH-1];

    reg [AW_X-1:0]   wptr;
    reg [AW_X-1:0]   fill;
    reg              primed;
    reg              wr_busy;
    reg [4:0]        wcnt;

    localparam integer REFRESH_SETS = N;
    reg [AW_N-1:0]   sets_since;
    reg              go_corr;

    //==================================================================
    //  Sample-set latch (29 words: NBAND ref bands + NBAND*NMIC mic bands
    //  + 1 broadband ref), drained over 29 clocks.
    //==================================================================
    reg signed [STORE_W-1:0] rh_full;
    reg signed [STORE_W-1:0] rh_b [0:NBAND-1];
    reg signed [STORE_W-1:0] xh   [0:NMIC-1][0:NBAND-1];

    // drain decode: word 0..3 = ref band, 4..27 = mic m band b, 28 = broadband
    wire [4:0]        dm   = wcnt - 5'd4;                       // 0..23 in mic range
    wire [MICSEL_W-1:0] dm_m = dm[MICSEL_W+BAND_W-1:BAND_W];   // bits [4:2]
    wire [BAND_W-1:0]   dm_b = dm[BAND_W-1:0];                 // bits [1:0]

    // Declared ahead of the latch generate below: Verilog-2001 requires a
    // declaration before use and ModelSim rejects a forward reference (the
    // same class as sk9822_ring.v's blank_run).
    reg [3:0] state;

    genvar glm, glb;
    generate
        for (glb = 0; glb < NBAND; glb = glb + 1) begin : g_latch_r
            always @(posedge clk)
                if (state == S_IDLE && !wr_busy && !go_corr && valid)
                    rh_b[glb] <= trunc_sat(ref_b[DATA_W*glb +: DATA_W]);
        end
        for (glm = 0; glm < NMIC; glm = glm + 1) begin : g_latch_x
            for (glb = 0; glb < NBAND; glb = glb + 1) begin : g_latch_xb
                always @(posedge clk)
                    if (state == S_IDLE && !wr_busy && !go_corr && valid)
                        xh[glm][glb] <= trunc_sat(mic_b[NBAND*DATA_W*glm + DATA_W*glb +: DATA_W]);
            end
        end
    endgenerate

    //==================================================================
    //  FSM registers
    //==================================================================
    reg [MICSEL_W-1:0]     mic_i;
    reg [BAND_W-1:0]       band_i;
    reg signed [3:0]       lag_i;
    reg [AW_N-1:0]         n_i;
    reg signed [ACC_W-1:0] acc;
    //  `best` now holds |C| (see PEAK_ABS).  UNSIGNED on purpose: a signed
    //  compare against a full-scale correlation magnitude would read a large
    //  |C| as a negative number and reject the very peak it should keep.
    reg [ACC_W-1:0]        best;
    reg signed [3:0]       best_lag;
    reg                    have_best;

    reg [ACC_W-1:0] E_x [0:NBAND-1];
    reg [ACC_W-1:0] E_r [0:NBAND-1];
    reg [ACC_W-1:0] e_ref;
    reg [5:0]       s_shift [0:NBAND-1];
    reg signed [ACC_W-1:0] R_acc;

    reg signed [ACC_W-1:0] cvals [0:NLAG-1];

    localparam [3:0] NLAG4   = NLAG;
    localparam [3:0] NLAG4M1 = NLAG - 1;

    wire [3:0] bidx  = LI2IDX(best_lag);
    wire [3:0] bidxm = (bidx == 4'd0)     ? 4'd0     : (bidx - 4'd1);
    wire [3:0] bidxp = (bidx == NLAG4M1)  ? NLAG4M1  : (bidx + 4'd1);

    wire signed [ACC_W-1:0] c_best  = cvals[bidx];
    wire signed [ACC_W-1:0] c_bestm = cvals[bidxm];
    wire signed [ACC_W-1:0] c_bestp = cvals[bidxp];

    wire signed [ACC_W+1:0] c_m_l = c_bestm;
    wire signed [ACC_W+1:0] c_0_l = c_best;
    wire signed [ACC_W+1:0] c_p_l = c_bestp;

    reg signed [ACC_W+1:0] num_q;
    reg signed [ACC_W+1:0] den_q;

    //==================================================================
    //  Restoring divider (unchanged).
    //==================================================================
    localparam integer DIV_N = FRAC_W + 3;
    localparam integer DVW       = ACC_W + FRAC_W + 4;
    localparam integer DIV_CNT_W = $clog2(DIV_N + 2);

    wire signed [DVW-1:0] num_x = $signed({{(DVW-ACC_W-2){num_q[ACC_W+1]}}, num_q});
    wire signed [DVW-1:0] den_x = $signed({{(DVW-ACC_W-2){den_q[ACC_W+1]}}, den_q});

    wire [DVW-1:0] num_abs = (num_x < 0) ? -num_x : num_x;
    wire [DVW-1:0] den_abs = (den_x < 0) ? -den_x : den_x;

    wire [DVW-1:0] rem_seed = num_abs <<< (FRAC_W-1);
    wire [DVW-1:0] dv_seed  = den_abs <<< (DIV_N-1);

    reg [DVW-1:0]       rem;
    reg [DVW-1:0]       dv;
    reg [FRAC_W+2:0]    quo;
    reg [DIV_CNT_W-1:0] div_i;

    localparam signed [FRAC_W+3:0] FRAC_MAX_Q =  (1 <<< FRAC_W) - 1;
    localparam signed [FRAC_W+3:0] FRAC_MIN_Q = -(1 <<< FRAC_W);

    wire signed [FRAC_W+3:0] quo_s    = $signed({1'b0, quo});
    wire signed [FRAC_W+3:0] frac_raw = (num_x < 0) ? quo_s : -quo_s;
    wire signed [FRAC_W+3:0] frac_sat = (frac_raw > FRAC_MAX_Q) ? FRAC_MAX_Q
                                      : (frac_raw < FRAC_MIN_Q) ? FRAC_MIN_Q
                                      : frac_raw;

    //==================================================================
    //  Memory addressing.
    //==================================================================
    wire signed [AW_X:0] XDEPTH_S   = XDEPTH;
    wire signed [AW_X:0] N_S        = N;
    wire signed [AW_X:0] x_base_raw = $signed({1'b0, wptr}) - N_S;

    function [AW_X-1:0] wrap_idx;
        input signed [AW_X:0] v;
        reg   signed [AW_X:0] a, r;
        begin
            a = (v < 0) ? (v + XDEPTH_S) : v;
            r = (a >= XDEPTH_S) ? (a - XDEPTH_S) : a;
            wrap_idx = r[AW_X-1:0];
        end
    endfunction

    wire signed [AW_X:0] xs_raw = x_base_raw + $signed({1'b0, n_i})
                                             + $signed(lag_i);
    wire signed [AW_X:0] rs_raw = x_base_raw + $signed({1'b0, n_i});
    wire [AW_X-1:0] xs_idx = wrap_idx(xs_raw);
    wire [AW_X-1:0] rs_idx = wrap_idx(rs_raw);

    // During S_ENERGY the mic read is at the unshifted window (lag 0); during
    // the correlation it is at base+n+lag.  rs_idx is base+n (lag-free).
    wire [AW_X-1:0] x_rd_idx = (state == S_ENERGY) ? rs_idx : xs_idx;

    reg signed [STORE_W-1:0] xrd, rrd, rrd_full;

    always @(posedge clk) begin
        xrd      <= xmem[XBASE(mic_i, band_i) + x_rd_idx];
        rrd      <= rmem[RBASE(band_i) + rs_idx];
        rrd_full <= rfull[rs_idx];
    end

    wire signed [STORE_W-1:0]   xs_s  = xrd      >>> SHIFT;
    wire signed [STORE_W-1:0]   rs_s  = rrd      >>> SHIFT;
    wire signed [STORE_W-1:0]   rf_s  = rrd_full >>> SHIFT;
    wire signed [2*STORE_W-1:0] prod  = $signed(xs_s) * $signed(rs_s);
    wire signed [2*STORE_W-1:0] rf2   = $signed(rf_s) * $signed(rf_s);
    // Normalisation energy uses sum-of-ABS (no multiplier; the golden model
    // measured it slightly BETTER than sum-of-squares).  e_ref keeps the
    // square rf2 because ANY_SND_E's scale must not move.
    wire [STORE_W-1:0]          xs_abs = (xs_s < 0) ? -xs_s : xs_s;
    wire [STORE_W-1:0]          rs_abs = (rs_s < 0) ? -rs_s : rs_s;

    wire signed [ACC_W-1:0] R_scaled = R_acc <<< NORM_G;

    //  Peak-search key (see the PEAK_ABS parameter).
    //    PEAK_ABS = 1 : |R_scaled| -- a polarity-inverted channel still
    //                   reports its TRUE lag, it just peaks downwards.
    //    PEAK_ABS = 0 : a negative R maps to 0 and can never win the compare,
    //                   which reproduces the old signed compare exactly.
    //  Two's complement negate of a 36-bit value is an adder, but it sits on
    //  a REGISTER-to-REGISTER path (R_acc is registered, best is registered),
    //  and the compare it feeds was already the critical cone -- measured
    //  slack after the change is in impl/pnr/audio.timing_paths.
    wire [ACC_W-1:0] R_mag = R_scaled[ACC_W-1]
                             ? (~R_scaled + {{(ACC_W-1){1'b0}}, 1'b1})
                             : R_scaled[ACC_W-1:0];
    wire [ACC_W-1:0] R_key = PEAK_ABS ? R_mag
                                      : (R_scaled[ACC_W-1] ? {ACC_W{1'b0}}
                                                           : R_scaled[ACC_W-1:0]);

    //==================================================================
    //  Single always block: write port + FSM.
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            mic_i      <= 0;
            band_i     <= 0;
            lag_i      <= -L;
            n_i        <= 0;
            acc        <= 0;
            best       <= 0;
            best_lag   <= 0;
            have_best  <= 1'b0;
            num_q      <= 0;
            den_q      <= 0;
            rem        <= 0;
            dv         <= 0;
            quo        <= 0;
            div_i      <= 0;
            done       <= 1'b0;
            any_valid  <= 1'b0;
            ref_energy <= 0;
            e_ref      <= 0;
            R_acc      <= 0;
            E_x[0] <= 0; E_x[1] <= 0; E_x[2] <= 0; E_x[3] <= 0;
            E_r[0] <= 0; E_r[1] <= 0; E_r[2] <= 0; E_r[3] <= 0;
            s_shift[0] <= 0; s_shift[1] <= 0; s_shift[2] <= 0; s_shift[3] <= 0;
            lag_flat   <= 0;
            frac_flat  <= 0;
            mag_flat   <= 0;
            valid_flat <= 0;
            wptr       <= 0;
            fill       <= 0;
            primed     <= 1'b0;
            wr_busy    <= 1'b0;
            wcnt       <= 0;
            go_corr    <= 1'b0;
            sets_since <= 0;
            rh_full    <= 0;
        end else begin
            done <= 1'b0;

            //----------------------------------------------------------
            //  HISTORY WRITE PORT (S_IDLE only)
            //----------------------------------------------------------
            if (state == S_IDLE) begin
                if (!wr_busy && !go_corr) begin
                    if (valid) begin
                        rh_full <= trunc_sat(ref_full);
                        wcnt    <= 5'd0;
                        wr_busy <= 1'b1;
                    end
                end else if (wr_busy) begin
                    if (wcnt < 5'd4) begin
                        rmem[RBASE(wcnt[BAND_W-1:0]) + wptr] <= rh_b[wcnt[BAND_W-1:0]];
                    end else if (wcnt < 5'd28) begin
                        xmem[XBASE(dm_m, dm_b) + wptr] <= xh[dm_m][dm_b];
                    end else begin
                        rfull[wptr] <= rh_full;
                    end

                    if (wcnt == 5'd28) begin
                        wr_busy <= 1'b0;
                        wptr <= (wptr == XDEPTH_LAST) ? {AW_X{1'b0}}
                                                      : (wptr + 1'b1);
                        if (fill == XDEPTH_LAST)
                            primed <= 1'b1;
                        else
                            fill <= fill + 1'b1;

                        if (!primed)
                            sets_since <= 0;
                        else if (sets_since == REFRESH_SETS-1) begin
                            sets_since <= 0;
                            go_corr    <= 1'b1;
                        end else
                            sets_since <= sets_since + 1'b1;
                    end else begin
                        wcnt <= wcnt + 5'd1;
                    end
                end
            end

            //----------------------------------------------------------
            //  CORRELATION FSM
            //----------------------------------------------------------
            case (state)
            S_IDLE: begin
                if (primed && !wr_busy && go_corr) begin
                    go_corr   <= 1'b0;
                    state     <= S_ENERGY;
                    mic_i     <= 0;
                    band_i    <= 0;
                    n_i       <= 0;
                    lag_i     <= 0;
                    e_ref     <= 0;
                    any_valid <= 1'b0;
                end
            end

            S_ENERGY: begin
                E_x[band_i] <= E_x[band_i] + {{(ACC_W-STORE_W){1'b0}}, xs_abs};
                if (mic_i == 0) begin
                    E_r[band_i] <= E_r[band_i] + {{(ACC_W-STORE_W){1'b0}}, rs_abs};
                    e_ref       <= e_ref       + {{(ACC_W-2*STORE_W){1'b0}}, rf2};
                end

                if (n_i == N_LAST) begin
                    n_i <= 0;
                    if (band_i == NBAND-1) begin
                        band_i <= 0;
                        state  <= S_SHIFT;
                    end else
                        band_i <= band_i + 1'b1;
                end else
                    n_i <= n_i + 1'b1;
            end

            S_SHIFT: begin
                s_shift[0] <= norm_shift(E_x[0], E_r[0]);
                s_shift[1] <= norm_shift(E_x[1], E_r[1]);
                s_shift[2] <= norm_shift(E_x[2], E_r[2]);
                s_shift[3] <= norm_shift(E_x[3], E_r[3]);
                E_x[0] <= 0; E_x[1] <= 0; E_x[2] <= 0; E_x[3] <= 0;
                lag_i   <= -L;
                band_i  <= 0;
                n_i     <= 0;
                acc     <= 0;
                R_acc   <= 0;
                have_best <= 1'b0;
                state   <= S_MAC;
            end

            S_MAC: begin
                acc <= acc + prod;
                if (n_i == N_LAST)
                    state <= S_NEXT;
                else
                    n_i <= n_i + 1'b1;
            end

            S_NEXT: begin
                R_acc <= R_acc + (acc >>> s_shift[band_i]);
                if (band_i == NBAND-1) begin
                    state <= S_LAG;
                end else begin
                    band_i <= band_i + 1'b1;
                    n_i    <= 0;
                    acc    <= 0;
                    state  <= S_WARM;
                end
            end

            //  One-cycle warm-up between bands: the read port is registered,
            //  so the first MAC cycle of a new band would otherwise consume
            //  the PREVIOUS band's last sample (a loud interferer in band 0
            //  leaking into band 1).  S_WARM lets the read settle on the new
            //  band before the MAC starts.  The cost is one duplicated sample
            //  per band (harmless), the win is no cross-band leakage.
            S_WARM: begin
                state <= S_MAC;
            end

            S_LAG: begin
                cvals[LI2IDX(lag_i)] <= R_scaled;
                if (have_best) begin
                    if (R_key > best) begin
                        best     <= R_key;
                        best_lag <= lag_i;
                    end
                end else begin
                    best      <= R_key;
                    best_lag  <= lag_i;
                    have_best <= 1'b1;
                end

                if (lag_i == L) begin
                    state <= S_INTERP;
                end else begin
                    lag_i <= lag_i + 1'b1;
                    band_i <= 0;
                    n_i    <= 0;
                    acc    <= 0;
                    R_acc  <= 0;
                    state  <= S_MAC;
                end
            end

            S_INTERP: begin
                if (best_lag == -L || best_lag == L) begin
                    num_q <= 0;
                    den_q <= 0;
                end else if (c_0_l[ACC_W+1]) begin
                    //  NEGATIVE peak -- only reachable with PEAK_ABS=1, i.e. a
                    //  polarity-inverted channel.  The parabola is fitted to
                    //  -C instead of C, so BOTH differences are negated: the
                    //  interpolated offset is identical, but den_q comes out
                    //  negative like the positive-peak case, which is what the
                    //  divider's sign convention (and the "flat top -> frac 0"
                    //  test below) assumes.  Without this the inverted
                    //  channels would silently fall back to integer-lag
                    //  accuracy, i.e. up to half a sample of extra residual.
                    num_q <= c_p_l - c_m_l;
                    den_q <= (c_0_l <<< 1) - c_m_l - c_p_l;
                end else begin
                    num_q <= c_m_l - c_p_l;
                    den_q <= c_m_l - (c_0_l <<< 1) + c_p_l;
                end
                div_i <= 0;
                state <= S_DIV;
            end

            S_DIV: begin
                if (div_i == 0) begin
                    rem   <= rem_seed;
                    dv    <= dv_seed;
                    quo   <= 0;
                    div_i <= 1;
                end else if (div_i > DIV_N) begin
                    state <= S_EMIT;
                end else begin
                    if (rem >= dv) begin
                        rem <= rem - dv;
                        quo <= (quo <<< 1) | 1'b1;
                    end else begin
                        quo <= quo <<< 1;
                    end
                    dv    <= dv >>> 1;
                    div_i <= div_i + 1'b1;
                end
            end

            S_EMIT: begin
                lag_flat[LAGIDX(mic_i) +: 4] <= best_lag;
                //  best already holds |C| (PEAK_ABS=1), and under PEAK_ABS=0
                //  it can only be 0 or a positive R -- either way the old
                //  `(best < 0) ? -best : best` was already a no-op here.
                mag_flat[ACCIDX(mic_i) +: ACC_W] <= best;
                valid_flat[mic_i] <= (best != 0);

                if (den_q >= -($signed(1) <<< (FRAC_W-2))) begin
                    frac_flat[FRACIDX(mic_i) +: (FRAC_W+1)] <= 0;
                end else begin
                    frac_flat[FRACIDX(mic_i) +: (FRAC_W+1)] <= frac_sat[FRAC_W:0];
                end

                if (best != 0)
                    any_valid <= 1'b1;

                ref_energy <= e_ref >>> REF_SH;

                if (mic_i == NMIC-1) begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end else begin
                    mic_i  <= mic_i + 1'b1;
                    band_i <= 0;
                    n_i    <= 0;
                    lag_i  <= 0;
                    state  <= S_ENERGY;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule


//-----------------------------------------------------------------------------
// tdoa_filter
//   Per-microphone low-pass on the fractional TDOA.  UNCHANGED from the
//   broadband engine: it speaks {lag * 2^FRAC_W + frac} and is agnostic to
//   how the engine produced that pair.  See the broadband tdoa_engine.v for
//   the full rationale (forgetting-factor mean, per-channel validity, slew
//   limit against 180-degree flips).
//-----------------------------------------------------------------------------
module tdoa_filter #(
    parameter integer NMIC   = 6,
    parameter integer FRAC_W = 10,
    parameter integer LAG_W  = 4,
    parameter integer SH     = 2,
    parameter integer SLEW   = 1024
) (
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        in_done,
    input  wire [LAG_W*NMIC-1:0]       in_lag,
    input  wire [(FRAC_W+1)*NMIC-1:0]  in_frac,
    input  wire [NMIC-1:0]             in_valid,
    input  wire                        in_any_valid,
    output reg                         out_done,
    output reg  [LAG_W*NMIC-1:0]       out_lag,
    output reg  [(FRAC_W+1)*NMIC-1:0]  out_frac,
    output reg  [NMIC-1:0]             out_valid,
    output reg                         out_any_valid
);
    wire step = in_done && in_any_valid;
    localparam integer TOT_W = LAG_W + FRAC_W + 1;
    localparam signed [TOT_W-1:0] SLIM = (SLEW == 0) ? {TOT_W{1'b0}}
                                                     : (SLEW <<< SH);
    reg signed [TOT_W-1:0] avg [0:NMIC-1];
    reg  [NMIC-1:0]        primed_m;

    genvar gi;
    generate
    for (gi = 0; gi < NMIC; gi = gi + 1) begin : g_lpf
        wire signed [LAG_W-1:0] lag_i = $signed(in_lag[LAG_W*gi +: LAG_W]);
        wire signed [FRAC_W:0]  fr_i  = $signed(in_frac[(FRAC_W+1)*gi +: (FRAC_W+1)]);
        wire signed [TOT_W-1:0] lag_x = {{(TOT_W-LAG_W){lag_i[LAG_W-1]}}, lag_i};
        wire signed [TOT_W-1:0] fr_x  = {{(TOT_W-FRAC_W-1){fr_i[FRAC_W]}}, fr_i};
        wire signed [TOT_W-1:0] x_now = (lag_x <<< FRAC_W) + fr_x;
        wire step_m = step && in_valid[gi];
        wire signed [TOT_W-1:0] d_x = x_now - avg[gi];
        wire signed [TOT_W-1:0] d_l = (SLEW == 0) ? d_x
                                    : (d_x >  SLIM) ?  SLIM
                                    : (d_x < -SLIM) ? -SLIM
                                    : d_x;
        wire signed [TOT_W-1:0] avg_nx = primed_m[gi]
                                       ? (avg[gi] + (d_l >>> SH))
                                       : x_now;
        wire signed [TOT_W-1:0] avg_step = step_m ? avg_nx : avg[gi];
        wire signed [TOT_W-1:0] lag_o = avg_step >>> FRAC_W;
        wire signed [TOT_W-1:0] fra_o = avg_step - (lag_o <<< FRAC_W);

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                avg[gi]      <= {TOT_W{1'b0}};
                primed_m[gi] <= 1'b0;
            end else if (step_m) begin
                avg[gi]      <= avg_nx;
                primed_m[gi] <= 1'b1;
            end
        end

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                out_lag [LAG_W*gi +: LAG_W]            <= {LAG_W{1'b0}};
                out_frac[(FRAC_W+1)*gi +: (FRAC_W+1)]  <= {(FRAC_W+1){1'b0}};
            end else if (step) begin
                out_lag [LAG_W*gi +: LAG_W]            <= lag_o[LAG_W-1:0];
                out_frac[(FRAC_W+1)*gi +: (FRAC_W+1)]  <= fra_o[FRAC_W:0];
            end
        end
    end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_done      <= 1'b0;
            out_valid     <= {NMIC{1'b0}};
            out_any_valid <= 1'b0;
        end else begin
            out_done      <= step;
            out_any_valid <= step;
            if (step)
                out_valid <= in_valid;
        end
    end

endmodule
