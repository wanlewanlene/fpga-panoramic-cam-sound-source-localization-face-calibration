//-----------------------------------------------------------------------------
// doa_solver.v     (rewritten 2026-10-11: channel calibration + robust fit)
//   Turns the six per-microphone TDOAs into a single bearing, and then into a
//   sector index for the 12-LED ring.
//
//   GEOMETRY
//   --------
//   Six microphones sit on a circle of radius R = 40 mm, 60 degrees apart,
//   microphone i at angle
//       phi_i = 90 - 60*i  degrees      (i = 0..5, clockwise from +Y)
//   with a seventh microphone at the centre acting as the TDOA reference.
//
//   A plane wave arriving from bearing theta reaches a ring microphone with
//   an extra path length of -(R) * <u, e_i>, so
//
//       tau_i = -(R/c) * cos(theta - phi_i)
//
//   INVERSION
//   ---------
//   Form the vector sum using  a_i = -(tau_i * c) / R = cos(theta - phi_i):
//       Sx = sum_i a_i * cos(phi_i),   Sy = sum_i a_i * sin(phi_i)
//   For six uniformly spaced microphones sum cos^2 = sum sin^2 = 3, hence
//       Sx = 3 cos(theta),  Sy = 3 sin(theta),  theta = atan2(Sy, Sx).
//   The common factor 3 cancels in atan2, so the design never needs a
//   division and never needs the exact c/(fs*R) constant: ANY fixed positive
//   scale gives the same bearing.
//
//   ACCURACY BUDGET -- read this before changing any of the numbers
//   ---------------------------------------------------------------
//   One sample of TDOA error is worth asin(c/(fs*R)) = 10.2 degrees at
//   R = 40 mm and fs = 48.83 kHz, and the whole aperture is only 5.6 samples
//   (R/c = 116.6 us).  Therefore:
//     * the sub-sample interpolator is NOT the accuracy limit -- 1/1024
//       sample is 0.01 degrees;
//     * what actually costs degrees is (a) one channel whose TDOA disagrees
//       with the other five, (b) correlation peaks smeared or displaced by
//       reverberation and by low-frequency noise that is coherent across the
//       array but is not a plane wave, (c) a fixed per-channel timing offset.
//   Three fixes live in this file and its neighbours:
//     * TRIM0..TRIM5: per-channel timing calibration, SUBTRACTED from each
//       channel's TDOA before the fit (parameter list below);
//     * a two-pass ROBUST fit: pass one is the plain least-squares solution,
//       then every microphone whose residual against that plane wave exceeds
//       R_MAX is dropped and the fit is repeated on the survivors.  One
//       reverberation-corrupted microphone would otherwise pull the bearing
//       by several degrees;
//     * the band-pass / pre-emphasis conditioning of mic_capture.v, which is
//       what makes the peaks themselves sharper and less reverb-prone.
//
//   SIGN (corrected 2026-10-16 -- the old note was right that the sign is
//         load-bearing and wrong about which way it pointed)
//   -----------------------------------------------------------------------
//   The polarity of a_i decides between theta and theta+180, and getting it
//   backwards is INVISIBLE in simulation whenever the testbench uses a
//   different convention from the hardware.  That is exactly what happened:
//   tb_doa_solver injected tau = -(R/c)cos(theta-phi) (this header's
//   convention) while tdoa_engine actually returns the correlation PEAK LAG,
//   which has the opposite sign for an early microphone.  The negation that
//   used to sit on a_q cancelled one of the two, so the board reported
//   theta+180: stand to one side of the array and the ring lights the far
//   side.
//
//   Ground truth, read off the engine instead of assumed:
//       tdoa_engine.v:  x_base_raw = wptr - N;
//                       xs_raw     = x_base_raw + n_i + lag_i;   (ring)
//                       rs_raw     = x_base_raw + n_i;           (reference)
//   lag = +1 therefore lines a LATER ring sample up with an EARLIER
//   reference sample -- the ring microphone is EARLY.  Hence
//       lag > 0  <=>  early,   and   a_i = +lag_i  ~  cos(theta - phi_i).
//
//   tb_doa_solver now injects the same sign the engine produces, so the two
//   conventions can no longer drift apart unnoticed.
//
//   Unchanged: the CORDIC quadrant pre-rotation.  It is a standard atan2 and
//   is sign-agnostic -- flipping both Sx and Sy moves the answer into the
//   correct half-plane by itself.
//
//   OUTPUT
//   ------
//   * bearing : signed Q1.16 fraction of pi, -65536..+65535 = -pi..+pi
//   * sector  : 0..NSEC-1, the LED ring position for that bearing
//   * confidence : 0..255 QUALITY, larger is better.  It is
//         255 * (1 - max|residual|/R_FULL) * (inliers/6), clamped.
//     This replaces the old "magnitude of the fitted vector", which was
//     absolutely scaled (so a quiet room always looked like a bad fit) and
//     could not distinguish a consistent fit from two microphones fighting.
//     A quiet but clean source now reports a HIGH quality, which is the
//     point: gating is about microphone agreement, not about loudness.
//   * resid_max : the largest residual of the surviving microphones, in
//     Q(FRAC_W) samples.  Feeds the UART diagnostic packet.
//
//   CORDIC vectoring mode computes atan2 without a divider.  12 iterations
//   leave a residual below 0.014 degrees, far finer than a 30 degree sector.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module doa_solver #(
    parameter integer NMIC   = 6,      // ring microphones
    parameter integer FRAC_W = 10,     // sub-sample fraction width (tdoa_engine)
    parameter integer VEC_W  = 24,     // width of the published quality field
    parameter integer NSEC   = 12,     // LED sectors around the ring
    // Mounting rotation of the LED ring, in DEGREES (signed).
    //
    // 2026-10-16: this used to live in top.v as an INTEGER number of sectors,
    // so it could only correct a whole 30 degree bin -- useless against the
    // 10..25 degree offset a hand-placed ring actually has.  It is a
    // continuous angle now, applied to the CORDIC result BEFORE the sector
    // quantiser, so any offset can be nulled to better than a degree.
    //
    // It deliberately does NOT move `bearing` or the diagnostic packet: those
    // stay in the MICROPHONE frame, because that is what a calibration
    // compares against a known source position.  Only the LED sector rotates.
    //
    // CALIBRATION: put a source at a known angle, read the B= field from the
    // UART status line (or S= if you prefer the 30 degree bin), then set this
    // to (true angle - reported angle), wrapped into +-180.
    parameter integer SECT_OFF_DEG = 0,

    //------------------------------------------------------------------
    // Per-channel timing trim, in Q(FRAC_W) samples.
    //
    // A POSITIVE trim means "this channel reports its TDOA too LATE by this
    // much", so the value is subtracted from that channel's tau before the
    // fit.  1 LSB = 1/1024 sample = 0.01 degrees of bearing; 1 sample is
    // 10.1 degrees (= asin(c/(fs*R)) at 48828.125 Hz, R = 40 mm), so a few
    // hundred LSBs is the range that matters.
    //
    // Calibrate with UART_PKT_MODE = 1 (see mic_uart_framer.v) and a source
    // at a known bearing.  With R = 40 mm, c = 343 m/s and the ACTUAL
    // fs = 48828.125 Hz (= 50 MHz / 16 / 64, NOT 48 kHz),
    //     R/(c*fs) = 5.694 samples
    //     tau_m(ideal) = -5.694 * cos(theta - phi_m)
    // so at theta = 0 the six ideal values are 0.00, -4.93, -4.93, 0.00,
    // +4.93, +4.93 samples.  Set TRIMm = (measured mean - ideal) * 1024;
    // tools/doa_diag.py --ref-theta <angle> prints exactly that table.
    // SECT_OFF in top.v stays for the mounting rotation; the two are
    // independent.
    parameter integer TRIM0 = 0,
    parameter integer TRIM1 = 0,
    parameter integer TRIM2 = 0,
    parameter integer TRIM3 = 0,
    parameter integer TRIM4 = 0,
    parameter integer TRIM5 = 0,

    //------------------------------------------------------------------
    // Robust fit.
    //   R_MAX  : a microphone whose residual against the pass-one plane wave
    //            exceeds this (Q(FRAC_W) samples) is treated as an outlier
    //            and dropped.  Default 1200 = 1.17 sample ~ 12 degrees.
    //
    //            2026-11-01: this used to be 350 (0.34 sample), justified by
    //            "a clean fit lands below 150".  That number came from the
    //            golden model's easy case, NOT from the per-microphone jitter
    //            the real front end produces: tools/sector_jitter.py measures
    //            sigma ~ 0.4 sample on a single channel, so a 0.34 sample gate
    //            rejected at least one microphone on ~90% of frames and left
    //            only 2-3 survivors on ~29% of them.  The survivor geometry
    //            correction (REFIT GEOMETRY) is a ratio of small popcounts, so
    //            on 2-3 microphones it amplifies the error instead of removing
    //            it, and the bearing jumps to the far side of the ring.
    //            Frame-level accuracy measured at 350: 52% correct, 29% more
    //            than one sector wrong.  At 1200: 93% correct, 0%.
    //   R_SH   : R_FULL = 2^R_SH is the residual at which quality reaches 0.
    //            Default 4096 = 4 samples.  It MUST stay above R_MAX: res_max
    //            is clamped to R_FULL, so a smaller R_FULL would turn every
    //            frame that only just passed the outlier gate into a
    //            confidence-0 frame -- the new gate would then reject exactly
    //            the frames the old one accepted.
    //------------------------------------------------------------------
    parameter integer R_MAX = 1200,
    parameter integer R_SH  = 12,

    //------------------------------------------------------------------
    // Array chirality.
    //
    // The microphone order around the circle is an ASSUMPTION taken from the
    // schematic (mic0..mic5 at phi = 90 - 60*i, clockwise), and it has never
    // been verified against a physical source.  If a source that walks
    // clockwise around the array makes the ring walk counter-clockwise, the
    // ring is mirrored: set MIRROR_X = 1 and the bearing is reflected about
    // the 90/270 degree axis.
    //
    // This is a REFLECTION of the fitted direction, not a rotation, which is
    // why SECT_OFF cannot fix it -- a rotation moves all twelve sectors
    // together, a mirror swaps them in pairs.
    //
    // FLIP_180 -- the POLARITY of the whole fit, 180 degrees.
    //
    // Set this when a source that is physically in front of microphone k is
    // reported as the direction OPPOSITE k (a rotation, not a mirror: every
    // bearing moves by exactly 180 and the twelve sectors keep their order).
    // It negates a_i = lag, i.e. it is exactly the sign that the 2026-10-16
    // note on `a_q` below argues about -- but that note was written against
    // tb_doa_solver, where the testbench drives the engine's own lag
    // convention.  ON THE BOARD the opposite convention has been measured, so
    // the two cannot both be right and this is where the difference is made
    // switchable instead of argued about.
    //
    // WHY THIS IS SAFE: res_pred is built from the same a_q, so res_err --
    // hence the inlier count, the confidence and the robust refit -- is
    // BIT-IDENTICAL.  Only the published bearing and the sector move.
    //------------------------------------------------------------------
    parameter integer MIRROR_X = 0,
    parameter integer FLIP_180 = 0
) (
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        tdoa_done,   // one pulse per TDOA set
    input  wire [4*NMIC-1:0]           lag_flat,
    input  wire [(FRAC_W+1)*NMIC-1:0]  frac_flat,
    // EFFECTIVE per-microphone validity: the engine's "a real peak was seen"
    // flag ANDed in top.v with the peak-magnitude guard (a microphone whose
    // peak is far below the others is unusable even when it is positive).
    input  wire [NMIC-1:0]             valid_flat,
    input  wire                        any_valid,

    output reg                         done,
    // bearing as a signed fraction of pi in Q1.16 (18-bit):
    //     -32768 .. +32767  <->  -pi/2 .. +pi/2
    //     -65536 .. +65535  <->  -pi   .. +pi     (whole circle)
    // 18 bits (not 17) because +pi maps to +32768 in Q1.15, which does not
    // fit a 17-bit signed value; using Q1.16 with 32768 <-> pi/2 keeps the
    // full circle exactly representable.
    output reg  signed [17:0]          bearing,
    output reg  [3:0]                  sector,
    output reg  [VEC_W-1:0]            confidence,  // 0..255 quality, big = good
    output reg  [15:0]                 resid_max    // Q(FRAC_W) samples
);

    localparam integer CORDIC_N = 12;
    localparam integer ACC_W    = 20;   // projection accumulator width
    localparam integer RF       = (1 <<< R_SH);   // residual full-scale

    //==================================================================
    //  Microphone direction constants, cos/sin held in Q1.14.
    //  phi_i = 90 - 60*i degrees.
    //==================================================================
    function signed [15:0] cos_phi;
        input [2:0] i;
        begin
            case (i)
                3'd0:    cos_phi =  16'sd0;      // cos( 90)
                3'd1:    cos_phi =  16'sd14189;  // cos( 30)
                3'd2:    cos_phi =  16'sd14189;  // cos(-30)
                3'd3:    cos_phi =  16'sd0;      // cos(-90)
                3'd4:    cos_phi = -16'sd14189;  // cos(-150)
                default: cos_phi = -16'sd14189;  // cos(-210)
            endcase
        end
    endfunction

    function signed [15:0] sin_phi;
        input [2:0] i;
        begin
            case (i)
                3'd0:    sin_phi =  16'sd16384;  // sin( 90) = 1
                3'd1:    sin_phi =  16'sd8192;   // sin( 30) = 0.5
                3'd2:    sin_phi = -16'sd8192;   // sin(-30)
                3'd3:    sin_phi = -16'sd16384;  // sin(-90) = -1
                3'd4:    sin_phi = -16'sd8192;   // sin(-150)
                default: sin_phi =  16'sd8192;   // sin(-210)
            endcase
        end
    endfunction

    // ---- geometry of the microphone set, derived from the MASK ----------
    //
    // The naive way to accumulate sum(cos^2)/sum(sin^2) is a per-microphone
    // lookup driven by `si` -- and that is what an earlier version of this
    // fix did.  It cost 1.6 ns of setup slack: `si` is already the head of
    // the solver's longest path (it indexes lag_flat / frac_flat / trim_sel
    // on the way to a_q and the projection multipliers), so adding two more
    // si-indexed LUTs lengthened that net.  Place & route measured it:
    // worst SETUP went from +2.342 ns to +0.703 ns.
    //
    // phi_i = 90 - 60*i makes the squares take only THREE values, so the sums
    // depend on the mask alone:
    //     cos^2 : 0 for mics 0,3;  14189^2 for mics 1,2,4,5
    //     sin^2 : 16384^2 for mics 0,3;  8192^2 for mics 1,2,4,5
    // which is a popcount over the mask -- no `si`, no accumulator, and the
    // result is latched once per pass instead of added six times.
    // (cos_phi is the rounded Q1.14 of cos(30 deg), so the four-fold term is
    // 5.6e-6 relative off the ideal 3*2^28; that is 1e-4 degrees of bearing.)
    //------------------------------------------------------------------
    localparam [31:0] COS2_60 = 32'd201327721;   // 14189^2  mics 1,2,4,5
    localparam [31:0] SIN2_90 = 32'd268435456;   // 16384^2  mics 0,3
    localparam [31:0] SIN2_30 = 32'd67108864;    //  8192^2  mics 1,2,4,5
    localparam integer GEO_SH = 20;              // scale before the multiply

    function [2:0] pc_a;                          // popcount of {1,2,4,5}
        input [5:0] m;
        begin
            pc_a = {2'b0, m[1]} + m[2] + m[4] + m[5];
        end
    endfunction

    function [2:0] pc_b;                          // popcount of {0,3}
        input [5:0] m;
        begin
            pc_b = {2'b0, m[0]} + m[3];
        end
    endfunction

    // scale down by 2^GEO_SH, with a guard so a degenerate set (only the two
    // microphones at +-90 deg, where cos = 0) cannot multiply an axis by zero
    // and pin the answer to exactly +-90 deg
    function [11:0] geo_gain;
        input [31:0] v;
        begin
            geo_gain = (v[31:GEO_SH] == 0) ? 12'd1 : v[31:GEO_SH];
        end
    endfunction

    wire [31:0] c2_p1_w = COS2_60 * pc_a(valid_flat);
    wire [31:0] s2_p1_w = (SIN2_90 * pc_b(valid_flat)) + (SIN2_30 * pc_a(valid_flat));

    // per-channel calibration trim, selected by the microphone index
    function signed [31:0] trim_sel;
        input [2:0] i;
        begin
            case (i)
                3'd0:    trim_sel = TRIM0;
                3'd1:    trim_sel = TRIM1;
                3'd2:    trim_sel = TRIM2;
                3'd3:    trim_sel = TRIM3;
                3'd4:    trim_sel = TRIM4;
                default: trim_sel = TRIM5;
            endcase
        end
    endfunction

    //==================================================================
    //  CORDIC arctangent table.
    //
    //  Angle format: Q2.16 with 32768 == pi/2, so 65536 == pi and the whole
    //  circle occupies -65536..+65536, which fits comfortably in 18 bits.
    //  (A 17-bit Q1.15 format cannot represent +pi, since pi maps to 32768,
    //  one past the largest 17-bit signed value -- that overflow was a
    //  genuine bug before this was changed.)
    //==================================================================
    function signed [17:0] atan_tab;
        input [3:0] i;
        begin
            case (i)
                4'd0 : atan_tab = 18'sd16384;
                4'd1 : atan_tab = 18'sd9672;
                4'd2 : atan_tab = 18'sd5110;
                4'd3 : atan_tab = 18'sd2594;
                4'd4 : atan_tab = 18'sd1302;
                4'd5 : atan_tab = 18'sd652;
                4'd6 : atan_tab = 18'sd326;
                4'd7 : atan_tab = 18'sd163;
                4'd8 : atan_tab = 18'sd81;
                4'd9 : atan_tab = 18'sd41;
                4'd10: atan_tab = 18'sd20;
                default: atan_tab = 18'sd10;
            endcase
        end
    endfunction

    //==================================================================
    //  Projection path.
    //
    //  CRITICAL PRECISION NOTE
    //  -----------------------
    //  The TDOA is at most 5.6 samples, and the sub-sample fraction carries a
    //  large share of the total information.  An earlier version computed
    //  a_q = (tau_q * 3) >> FRAC_W, which rounds the value to an INTEGER and
    //  throws the fraction away entirely (a fraction of 0.9 sample was lost
    //  outright), producing a systematic 2-5 degree bearing error.  The
    //  accumulator must therefore stay in Q(FRAC_W) all the way through.
    //
    //  The arbitrary factor 3 has been removed: atan2 is invariant to a
    //  positive common scale, so the constant is pointless and only wastes
    //  bits.  a_q is simply the calibrated, negated tau in Q(FRAC_W) units.
    //
    //  SIGN EXTENSION -- this is where an earlier version went wrong.
    //
    //  The lag field is FOUR bits wide, and negative lags are stored in two's
    //  complement (e.g. -1 is 4'b1111).  Declaring the intermediate as
    //     wire signed [15:0] lag_v = $signed(lag_flat[4*si +: 4]);
    //  does NOT sign-extend: Verilog sizes the part-select first (4 bits),
    //  then zero-extends it to the declared 16 bits, so 4'b1111 became
    //  +15 instead of -1.  The fix is to take the 4-bit slice, sign-extend it
    //  EXPLICITLY to 32 bits, and only then re-extend by FRAC_W.
    //==================================================================
    reg  [2:0]             si;
    reg  signed [ACC_W-1:0] sx, sy;      // pass 1 (all valid microphones)
    reg  signed [ACC_W-1:0] sx2, sy2;    // pass 2 (inliers only)

    //==================================================================
    //  REFIT GEOMETRY -- WHY THESE REGISTERS EXIST
    //
    //  atan2(Sy, Sx) only recovers the bearing because six uniformly spaced
    //  microphones give, as an identity,
    //        sum cos^2(phi_i) = sum sin^2(phi_i) = 3
    //  so the common factor 3 cancels between the two axes.  Drop a
    //  microphone and the two sums are no longer equal: the ratio Sy/Sx is
    //  then scaled by the wrong amount and the reported bearing is pulled
    //  towards the X axis.
    //
    //  Measured on the real RTL before this fix (tb_doa_solver, test 5):
    //        survivors {0,1,2,4,5}  (sum cos^2 = 3, sum sin^2 = 2)
    //            theory atan2(2 sin15, 3 cos15) = 10.12 deg, hardware 10.10
    //        survivors {1,2,4,5}    (sum cos^2 = 3, sum sin^2 = 1)
    //            theory 5.10 deg, hardware 5.12
    //  i.e. up to 26.6 deg of error, while confidence stays above CONF_MIN --
    //  the ring LIGHTS UP AND POINTS THE WRONG WAY, which is worse than a
    //  dark ring.  The robust pass is supposed to improve the answer; without
    //  this normalisation it is the one thing that breaks it.
    //
    //  Fix: feed the CORDIC
    //        atan2( Sy * sum cos^2 ,  Sx * sum sin^2 )
    //  which is the diagonal solution of the 2x2 normal equations (exact when
    //  the survivor set stays symmetric about the X axis, which is the common
    //  case here).  Pass 1 needs it too -- the SAME imbalance occurs whenever
    //  valid_flat masks a channel.
    //
    //  Both gains are LATCHED once, at the end of their pass, so the CORDIC
    //  input sees a register (see the popcount note on c2_p1_w above).
    //==================================================================
    reg  [11:0] c2_q1, s2_q1;            // pass 1 gains (valid_flat geometry)
    reg  [11:0] c2_q2, s2_q2;            // pass 2 gains (survivor geometry)

    wire [31:0] c2_p2_w = COS2_60 * pc_a(inl);
    wire [31:0] s2_p2_w = (SIN2_90 * pc_b(inl)) + (SIN2_30 * pc_a(inl));

    wire        [3:0]      lag_raw = lag_flat[4*si +: 4];
    wire signed [31:0]     lag_se  = $signed({{28{lag_raw[3]}}, lag_raw});

    wire        [FRAC_W:0] fr_raw = frac_flat[(FRAC_W+1)*si +: (FRAC_W+1)];
    wire signed [31:0]     fr_se  = $signed({{(31-FRAC_W){fr_raw[FRAC_W]}}, fr_raw});

    // tau in Q(FRAC_W) sample units, calibrated.
    //
    // POLARITY -- fixed 2026-10-16, this was the "ring lights the far side"
    // ------------------------------------------------------------------
    // tdoa_engine correlates  sum_n x[base+n+lag] * r[base+n]  with
    // x_base_raw = wptr - N, so lag = +1 reads a sample one LATER in the
    // ring's own history than the reference reads.  Aligning a later ring
    // sample with an earlier reference sample means the ring microphone
    // arrived EARLIER, so
    //        lag > 0   <=>   that microphone is EARLY.
    // The plane-wave model wants a_i = cos(theta - phi_i), and the engine
    // therefore already hands us exactly that (up to a fixed positive
    // scale, which atan2 ignores).  So a_i IS the lag as it stands.
    //
    // The negation that used to sit on this line was written against the
    // header's tau = -(R/c)cos(theta-phi) convention ("negative = early")
    // rather than against the engine's actual output, and it rotated every
    // bearing by exactly 180 degrees: the board lit the LED on the OPPOSITE
    // side of the talker while the simulation stayed green, because
    // tb_doa_solver injected tau with the header's sign.
    //
    // res_pred is formed from the same sx/sy, so both sides of res_err flip
    // together and |res_err| -- hence the whole robust pass -- is unchanged.
    wire signed [31:0] tau_q = (lag_se <<< FRAC_W) + fr_se;
    wire signed [31:0] tau_c = tau_q - trim_sel(si);

    //  FLIP_180 does NOT live here.  Tried it on 2026-11-01 and place & route
    //  went from +0.94 ns to -2.10 ns over 25 paths: this signal feeds the
    //  per-microphone projection multipliers AND the residual subtract, so a
    //  32-bit negation on it lands in the middle of the solver's longest
    //  combinational cone (the report named u_doa/a_q_* as the start point).
    //  It is applied at the CORDIC pre-rotation instead -- see S_PRE, where
    //  it costs one constant selector and nothing else.
    wire signed [31:0] a_q   = tau_c;

    //------------------------------------------------------------------
    //  PROJECTION PIPELINE  (2026-11-01)
    //
    //  ax/ay used to be
    //      ax = (a_q * cos_phi(si)) >>> 14
    //  with a_q built in the same cycle from three si-indexed lookups
    //  (lag_flat / frac_flat variable slices and trim_sel(si)).  That gave
    //  one cone of
    //      si -> frac barrel shift -> trim mux -> subtract -> coef mux ->
    //      multiply -> 32-bit add -> sx
    //  which place & route measured at 23.5..24.3 ns, i.e. -0.86 ns, after
    //  an unrelated change in top.v shifted the floorplan.  An hour earlier
    //  the same logic had placed at 19.2 ns, so this cone was one placement
    //  away from failing at all times -- not something to keep re-rolling.
    //
    //  The barrel/mux/subtract half only depends on `si`, and `si` is held
    //  for as long as the microphone is being processed, so it can be
    //  REGISTERED instead of re-derived.  S_ACC and S_FIT2 now take two
    //  clocks per microphone: the first latches a_p / c_p / s_p, the second
    //  does the multiply and the accumulate.  Cost: 12 clocks per pass
    //  instead of 6, out of a ~5.6 ms pass period -- nothing.
    //
    //  The registers are declared next to res_ph; the shift is the same one
    //  as before, so the arithmetic is bit-for-bit unchanged.
    //------------------------------------------------------------------
    //  Pipeline registers.  Declared HERE, ahead of ax/ay, for the reason the
    //  file keeps rediscovering: Verilog-2001 needs a name declared before it
    //  is used, and a forward reference is a hard ModelSim error (vlog-2730)
    //  even when Gowin's synthesiser quietly accepts it.
    //
    //  acc_ph is the sub-phase shared by S_ACC and S_FIT2 -- 0 latches this
    //  microphone's delay and coefficients, 1 multiplies and accumulates.
    //  It is cleared whenever either loop is entered, so both can share it.
    reg                acc_ph;
    reg  signed [31:0] a_p;
    reg  signed [15:0] c_p, s_p;

    wire signed [15:0] a_pn = a_p[15:0];   // |a_q| < 2^13, see the bound
                                           // below -- 16 bits is exact

    // constants are Q1.14, so this shift produces the projection in Q(FRAC_W)
    wire signed [31:0] ax = (a_pn * c_p) >>> 14;
    wire signed [31:0] ay = (a_pn * s_p) >>> 14;

    //  BOUND (the whole safety argument for a_pn/a_p[15:0]):
    //      |a_q| = |lag*2^FRAC_W + frac - trim|
    //            <=  L*1024 + 1024 + |trim|  ~= 3.3k  <  2^15
    //  with tdoa_engine's L = 2 and FRAC_W = 10.  If either grows, widen
    //  a_pn.  tb_doa_solver pins the bearings, so a silent overflow fails
    //  the build rather than the board.

    //==================================================================
    //  Residual of microphone si against the plane wave fitted so far.
    //
    //  The model predicts   a_i = (Sx*cos(phi_i) + Sy*sin(phi_i)) / 3
    //  (the 1/3 undoes the factor of three that six uniformly spaced
    //  microphones produce -- see INVERSION above).  1/3 is applied as a
    //  multiply by 21846/65536, exact to 1e-5.
    //
    //  |Sx| <= 2^16 and |cos| <= 2^14, so the products fit 31 bits; the
    //  multiply by 21846 is done at 40 bits because 2^17 * 21846 does not
    //  fit a 32-bit signed value and a truncated product here would corrupt
    //  the residual silently.
    //==================================================================
    wire signed [31:0] pcos = (sx * $signed(cos_phi(si))) >>> 14;
    wire signed [31:0] psin = (sy * $signed(sin_phi(si))) >>> 14;
    wire signed [31:0] psum = pcos + psin;

    //  S_RES is run in THREE registered sub-phases per microphone (see the
    //  FSM below): phase 0 captures the projection sum, phase 1 the
    //  plane-wave prediction, phase 2 performs the classification.
    //
    //  This used to be a single cycle.  The 2026-10-11 place & route showed
    //  the resulting cone -- two 32-bit multiplies, a 40-bit multiply, a
    //  subtract, an absolute value and a compare -- leaving only +0.153 ns
    //  of margin, which is not a safe number against supply and temperature
    //  drift.  Splitting the multipliers apart costs 18 clk instead of 6
    //  out of a 5.6 ms pass period, i.e. nothing.
    reg  [1:0]         res_ph;      // S_RES sub-phase, 0..2
    reg  signed [31:0] res_psum;
    reg  signed [31:0] res_pred;
    wire signed [39:0] res_pred_w = ($signed(res_psum) * 40'sd21846) >>> 16;
    wire signed [31:0] res_err    = a_q - res_pred;
    wire        [31:0] errm_r     = (res_err < 0) ? -res_err : res_err;

    reg  [5:0]  inl;          // survivors of the robust pass
    reg  [19:0] res_max;      // largest residual among the survivors

    // inlier count, evaluated combinationally so the final microphone of the
    // second pass is already counted when the FSM leaves it
    wire [3:0] ninl_w = {3'b0, inl[0]} + {3'b0, inl[1]} + {3'b0, inl[2]}
                      + {3'b0, inl[3]} + {3'b0, inl[4]} + {3'b0, inl[5]};

    // fit chosen for the CORDIC: the robust one, unless too few microphones
    // survived to determine two unknowns
    wire                    robust = (ninl_w >= 4'd3);
    wire signed [ACC_W-1:0] fx_raw = robust ? sx2 : sx;
    wire signed [ACC_W-1:0] fy     = robust ? sy2 : sy;

    // Array chirality (MIRROR_X): a mirrored microphone order reflects the
    // fitted direction about the Y axis, which is exactly a negation of Sx.
    // Applied here, and NOT in the pre-rotation, so that the residual
    // classification above keeps working against the self-consistent fit --
    // a mirror changes the reported angle, never the residuals.
    wire signed [ACC_W-1:0] fx = MIRROR_X ? -fx_raw : fx_raw;

    //------------------------------------------------------------------
    //  Geometry normalisation (see the REFIT GEOMETRY note above).
    //
    //  c2_q1/c2_q2 are registers latched at the end of each pass, so the
    //  CORDIC input path is
    //      register -> multiply -> sign mux -> register
    //  with no mask logic, no popcount and no `si` lookup in it.
    //
    //  WIDTH RULE: a Verilog multiply is evaluated at the width of its
    //  LARGEST operand, so `fx * 12'bit_gain` would silently compute at 20
    //  bits and truncate the product.  The gains are therefore widened to 32
    //  bits explicitly -- the same trap res_pred_w avoids by casting 21846 to
    //  40 bits.  |fx| <= 6 * 5837 = 35022, so the real product never passes
    //  35022 * 1535 = 5.4e7 (27 bits) and fits 32 with room to spare.
    //------------------------------------------------------------------
    wire [11:0]        c2_q = robust ? c2_q2 : c2_q1;
    wire [11:0]        s2_q = robust ? s2_q2 : s2_q1;
    wire signed [31:0] c2_g = $signed({20'b0, c2_q});
    wire signed [31:0] s2_g = $signed({20'b0, s2_q});

    // num = Sy * sum cos^2   den = Sx * sum sin^2
    wire signed [31:0] fx_n = fx * s2_g;
    wire signed [31:0] fy_n = fy * c2_g;

    //==================================================================
    //  Quality  (0..255, larger is better)
    //    255 * (RF - res_max)/RF, scaled by inliers/6
    //  The scale factor is 43/256 ~ 1/6, so six inliers leave it untouched
    //  and three halve it.
    //
    //  WHAT THE NUMBER MEANS -- this is the identity to use when tuning a
    //  gate, and it is easy to get wrong by intuition:
    //    res_max is the largest residual AMONG THE INLIERS, and an inlier is
    //    by definition within R_MAX, so res_max <= R_MAX always.  With the
    //    defaults (R_MAX = 1200 = 1.17 sample, RF = 4096 = 4 samples) the
    //    quality is therefore bounded at min(R_MAX, RF) = 1.17 sample:
    //        6 inliers -> 180 .. 255      5 -> 150 .. 212
    //        4 inliers -> 120 .. 170      3 ->  90 .. 127
    //    i.e. THREE inliers can never reach 128, and four always can.  That
    //    is what CONF_MIN = 128 in top.v means: "at least four microphones
    //    agree on one plane wave".  The other way to make a gate pass more
    //    often is to raise R_MAX (widen the tolerance); neither can rescue a
    //    wrong bearing.
    //==================================================================
    wire [19:0] res_l  = (res_max > RF[19:0]) ? RF[19:0] : res_max;

    //------------------------------------------------------------------
    //  QUALITY ARITHMETIC -- written as shift/add on purpose (2026-11-01)
    //
    //  This was  ((RF-res) * 255) >>> R_SH  then  (* ninl * 43) >>> 8, and
    //  synthesis built THREE cascaded multipliers (mult_659 -> n1322 ->
    //  mult_673) between res_max and the confidence register.  The place &
    //  route report put that path at 24.7 ns against a 24.5 ns requirement,
    //  i.e. the whole design's worst setup slack was decided by a block of
    //  constant multiplications.  Both constants are power-of-two-ish, so
    //  neither needs a multiplier at all:
    //
    //      x * 255    = (x << 8) - x   -- one subtractor
    //      ninl * 43  = a seven-entry table -- ninl is only 0..6
    //
    //  The value is IDENTICAL, term for term: integer multiplication is
    //  associative and nothing is truncated in between, so tb_doa_solver's
    //  quality checks must pass unchanged.
    //------------------------------------------------------------------
    wire [19:0] res_d   = RF[19:0] - res_l;                 // 0 .. RF
    wire [31:0] res_x   = {12'd0, res_d};                   // widen, 32 bits
    wire [31:0] qual_r  = ((res_x << 8) - res_x) >>> R_SH;  // *255, then scale

    //  NARROW OPERANDS (2026-11-01).  qual_r is <= 255 by construction
    //  ((RF*255)>>R_SH = 255 when res_l = 0), so the product below only ever
    //  needs 8 x 9 bits.  Written as 32-bit wires -- the obvious way, and what
    //  this was -- GowinSynthesis built a 32x32 multiplier, and place & route
    //  reported that single multiplier as the worst setup path in the whole
    //  design (24.2 ns of a 24.5 ns budget).  Narrowing the declared widths is
    //  a pure declaration change: the arithmetic value is identical.
    wire [7:0]  qual_q  = qual_r[7:0];                      // 0 .. 255
    wire [12:0] n43w    = ninl_w * 9'd43;                   // 0 .. 258, exact width
    wire [20:0] qual_p  = (qual_q * n43w) >>> 8;            // 0 .. 257
    wire [15:0] qual_c = (qual_p > 21'd255) ? 16'd255 : qual_p[15:0];

    //==================================================================
    //  Control
    //==================================================================
    localparam integer S_IDLE = 3'd0,
                       S_ACC  = 3'd1,
                       S_RES  = 3'd2,
                       S_FIT2 = 3'd3,
                       S_PRE  = 3'd4,
                       S_ROT  = 3'd5,
                       S_PUB  = 3'd6;

    reg [2:0]        state;
    reg [3:0]        ci;
    reg signed [31:0] cx, cy;
    reg signed [17:0] cz;          // accumulated angle, Q2.16 (32768 == pi/2)

    // Widen the published angle before the +-pi fold: the fold subtracts
    // 131072, which does not fit an 18-bit register, so the arithmetic is
    // carried out at 20 bits and narrowed only on the final assignment.
    wire signed [19:0] cz_w  = {{2{cz[17]}}, cz};
    wire signed [19:0] cz_f  = (cz_w > 20'sd65536) ? (cz_w - 20'sd131072) : cz_w;

    //  Clamp to +-pi, then APPLY THE MOUNTING ROTATION.
    //
    //  The rotation is placed here, between the fold and the quantiser, for
    //  two reasons: it has to be continuous (an integer-sector offset cannot
    //  express a 15 degree misalignment), and it must NOT leak into
    //  `bearing`, which is published in the microphone frame for calibration.
    //
    //  1 degree = 65536/180 = 364.09 LSB; SECT_OFF_Q keeps the integer
    //  rounding of that conversion.
    localparam signed [19:0] SECT_OFF_Q = (SECT_OFF_DEG * 65536) / 180;

    wire signed [19:0] cz_lat = (cz_f < -20'sd65536) ? -20'sd65536
                              : (cz_f >  20'sd65536) ?  20'sd65536
                              : cz_f;

    wire signed [19:0] cz_sh  = cz_lat + SECT_OFF_Q;
    wire signed [19:0] cz_sec = (cz_sh >=  20'sd65536) ? cz_sh - 20'sd131072
                              : (cz_sh <  -20'sd65536) ? cz_sh + 20'sd131072
                              : cz_sh;

    // current CORDIC table entry, evaluated combinationally from ci
    wire signed [17:0] atan_now = atan_tab(ci);

    // Sector index.  (cz_sec + pi) lies in 0..2^17 and NSEC <= 12 < 2^4, so
    // the product needs 21 bits; evaluated at 24 bits so nothing has to be
    // narrowed implicitly, then the shift by 17 (divide by 2pi) leaves 0..12.
    localparam [3:0] NSEC4 = NSEC;
    wire [23:0] sector_calc = (((cz_sec + 20'sd65536) * NSEC4) >>> 17);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            si         <= 0;
            sx         <= 0;
            sy         <= 0;
            sx2        <= 0;
            sy2        <= 0;
            c2_q1      <= 12'd1;
            s2_q1      <= 12'd1;
            c2_q2      <= 12'd1;
            s2_q2      <= 12'd1;
            inl        <= 6'd0;
            res_max    <= 20'd0;
            res_ph     <= 2'd0;
            res_psum   <= 32'sd0;
            res_pred   <= 32'sd0;
            acc_ph     <= 1'b0;
            a_p        <= 32'sd0;
            c_p        <= 16'sd0;
            s_p        <= 16'sd0;
            ci         <= 0;
            cx         <= 0;
            cy         <= 0;
            cz         <= 0;
            done       <= 1'b0;
            bearing    <= 0;
            sector     <= 0;
            confidence <= 0;
            resid_max  <= 16'd0;
        end else begin
            done <= 1'b0;

            case (state)
            //----------------------------------------------------------
            S_IDLE: begin
                if (tdoa_done && any_valid) begin
                    state   <= S_ACC;
                    si      <= 0;
                    sx      <= 0;
                    sy      <= 0;
                    sx2     <= 0;
                    sy2     <= 0;
                    // the geometry gains are latched at the end of each pass
                    // (see REFIT GEOMETRY); reset them to 1 = "no correction"
                    // so a pass that never completes cannot scale the fit by 0
                    c2_q1   <= 12'd1;
                    s2_q1   <= 12'd1;
                    c2_q2   <= 12'd1;
                    s2_q2   <= 12'd1;
                    // res_max accumulates a maximum across the pass, so it
                    // MUST be cleared here -- leaving it to the reset alone
                    // would latch the worst residual ever seen and pin the
                    // quality at its floor for the rest of the session.
                    res_max <= 20'd0;
                    res_ph  <= 2'd0;
                    acc_ph  <= 1'b0;   // start pass 1 on its "latch" half
                end
            end

            //----------------------------------------------------------
            // Pass 1: two clocks per microphone.
            //   acc_ph 0: latch this microphone's delay and its coefficients
            //   acc_ph 1: multiply, then accumulate into sx / sy
            // Splitting here is what keeps the si-indexed lookups out of the
            // multiply's cone -- see PROJECTION PIPELINE at ax/ay.
            // Microphones the engine flagged invalid contribute nothing.
            S_ACC: begin
                if (!acc_ph) begin
                    a_p    <= a_q;
                    c_p    <= cos_phi(si);
                    s_p    <= sin_phi(si);
                    acc_ph <= 1'b1;
                end else begin
                    if (valid_flat[si]) begin
                        sx  <= sx + ax[ACC_W-1:0];
                        sy  <= sy + ay[ACC_W-1:0];
                    end
                    acc_ph <= 1'b0;

                    // si must be REWOUND here, not left at NMIC-1.  S_RES is a
                    // per-microphone scan of its own, and if it is entered with
                    // si already at NMIC-1 it runs exactly once and classifies
                    // only the LAST microphone: inl[0..4] then stay zero, the
                    // inlier count ninl_w can never exceed 1, the robust refit is
                    // silently skipped (fx_raw always takes sx) and the quality
                    // term is multiplied by 1/6, which parks confidence below
                    // CONF_MIN forever.  Found by review 2026-10-11.
                    if (si == NMIC-1) begin
                        // latch the pass-1 geometry gain (mask-only, so it is
                        // stable here and independent of si)
                        c2_q1 <= geo_gain(c2_p1_w);
                        s2_q1 <= geo_gain(s2_p1_w);
                        state <= S_RES;
                        si    <= 3'd0;
                    end else
                        si <= si + 1'b1;
                end
            end

            //----------------------------------------------------------
            // Classify every microphone against the pass-1 plane wave and
            // keep the largest residual among the survivors for the quality
            // figure.
            //
            // Three sub-phases per microphone, selected by res_ph:
            //   0: capture the projection sum psum (two multiplies + add)
            //   1: capture the plane-wave prediction (40-bit multiply)
            //   2: residual, absolute value, compare, and advance si
            //
            // si is held across all three sub-phases, so a_q / cos_phi(si) /
            // sin_phi(si) are stable wherever they are read.  It is rewound
            // to 0 on the hand-off to S_FIT2 for the reason documented in
            // S_ACC above.
            //----------------------------------------------------------
            S_RES: begin
                case (res_ph)
                2'd0: begin
                    res_psum <= psum;
                    res_ph   <= 2'd1;
                end
                2'd1: begin
                    res_pred <= res_pred_w[31:0];
                    res_ph   <= 2'd2;
                end
                default: begin
                    // R_MAX is an integer parameter and must stay POSITIVE:
                    // the comparison is unsigned because errm_r is unsigned,
                    // so a negative R_MAX would read as a huge threshold.
                    inl[si] <= valid_flat[si] && (errm_r <= R_MAX);

                    if (valid_flat[si] && (errm_r <= R_MAX) && (errm_r > res_max))
                        res_max <= errm_r[19:0];

                    res_ph <= 2'd0;
                    if (si == NMIC-1) begin
                        state  <= S_FIT2;
                        si     <= 3'd0;
                        acc_ph <= 1'b0;    // pass 2 starts on its "latch" half
                    end else
                        si <= si + 1'b1;
                end
                endcase
            end

            //----------------------------------------------------------
            // Pass 2: refit on the survivors only, same two-clock split as
            // S_ACC (acc_ph was cleared on the hand-off from S_RES).
            S_FIT2: begin
                if (!acc_ph) begin
                    a_p    <= a_q;
                    c_p    <= cos_phi(si);
                    s_p    <= sin_phi(si);
                    acc_ph <= 1'b1;
                end else begin
                    if (inl[si]) begin
                        sx2 <= sx2 + ax[ACC_W-1:0];
                        sy2 <= sy2 + ay[ACC_W-1:0];
                    end
                    acc_ph <= 1'b0;

                    if (si == NMIC-1) begin
                        // survivor geometry: without it atan2(Sy2,Sx2) is scaled
                        // by the wrong ratio and the bearing is pulled towards
                        // the X axis (see REFIT GEOMETRY at the declarations).
                        // `inl` is complete by now -- it was written during S_RES.
                        c2_q2 <= geo_gain(c2_p2_w);
                        s2_q2 <= geo_gain(s2_p2_w);
                        state <= S_PRE;
                    end else
                        si <= si + 1'b1;
                end
            end

            //----------------------------------------------------------
            // Pre-rotation: the CORDIC vectoring iteration only converges for
            // vectors in the right half plane, so a vector pointing left is
            // negated and pi is added to the accumulated angle.  This runs as
            // its own state because the projection accumulator has not yet
            // settled when the last microphone is consumed.
            S_PRE: begin
                // fx_n / fy_n are the geometry-normalised fit (see the REFIT
                // GEOMETRY note).  Their signs still say which quadrant the
                // vector is in, because the gains are strictly positive.
                //  FLIP_180 adds a further pi to the accumulated angle.  Both
                //  branches already load a CONSTANT into cz, so this is a
                //  selector between two literals -- no adder, no comparator,
                //  nothing on the path.  Rotating the start vector by pi is
                //  the same as negating it, which is why adding pi to the
                //  result is equivalent to negating a_i upstream, but free.
                if (fx_n < 0) begin
                    cx <= -fx_n;
                    cy <= -fy_n;
                    cz <= FLIP_180 ? 18'sd0 : 18'sd65536;      // pi + pi == 0
                end else begin
                    cx <=  fx_n;
                    cy <=  fy_n;
                    cz <= FLIP_180 ? 18'sd65536 : 18'sd0;      // +pi
                end
                ci    <= 0;
                state <= S_ROT;
            end

            //----------------------------------------------------------
            S_ROT: begin
                // atan_tab() is a function, and Verilog-2001 forbids taking
                // a bit-select of a function result, so the value is read
                // into a named signal and sign-extended from there.  It is a
                // wire (not a register) precisely so that the add/subtract
                // below consumes the CURRENT iteration's table entry rather
                // than the previous one.
                if (cy >= 0) begin
                    cx <= cx + (cy >>> ci);
                    cy <= cy - (cx >>> ci);
                    cz <= cz + $signed(atan_now);
                end else begin
                    cx <= cx - (cy >>> ci);
                    cy <= cy + (cx >>> ci);
                    cz <= cz - $signed(atan_now);
                end

                if (ci == CORDIC_N-1)
                    state <= S_PUB;
                else
                    ci <= ci + 1'b1;
            end

            //----------------------------------------------------------
            S_PUB: begin
                // cz is atan2 in Q2.16 (32768 == pi/2), accumulated with a
                // +pi offset applied whenever the pre-rotation negated the
                // vector.  That offset means the raw value can land in
                // (pi, 3pi/2], which represents a true bearing in
                // (-pi, -pi/2].  Fold those back by subtracting 2pi so the
                // published bearing always lies in -pi..+pi:
                //
                // cz = 65551  (180.04 deg)  ->  -65521  (-179.96 deg)
                //     cz = 98309  (270.01 deg)  ->  -32763  ( -89.99 deg)
                //
                // Without this fold, sector 0..3 (the lower-left quadrant)
                // was unreachable and those bearings were clamped to the last
                // sector instead -- 23 of 72 test vectors were wrong.
                //
                // The sum below can differ from cz by at most 2^17, so the
                // interim is computed at 20 bits; cz_sec is cz_f clamped to
                // +-pi so a bearing of exactly +pi cannot index one sector
                // past the end of the ring.
                bearing <= cz_f[17:0];

                // sector = floor((theta + pi) / (2pi) * NSEC), evaluated on
                // the FOLDED angle so the bin edges line up with the LED ring
                // (bin 0 starts at -pi).
                if (cz_sec >= 20'sd65536)
                    sector <= NSEC - 1;
                else if (cz_sec <= -20'sd65536)
                    sector <= 4'd0;
                else
                    sector <= sector_calc[3:0];

                confidence <= qual_c;
                resid_max  <= res_l[15:0];

                done  <= 1'b1;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
