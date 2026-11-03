//===========================================================================
//  first_arrival.v  --  "which microphone heard it FIRST", one report per event
//
//  WHAT IT ANSWERS
//    Seven microphones sample the same room at the same time.  A sound reaches
//    one of them a few tens of microseconds before the others; with R = 40 mm
//    the largest possible arrival difference is R/c = 116.6 us = 5.7 samples
//    at 48.83 kHz.  tdoa_engine measures that difference per ring microphone
//    against the centre reference, so:
//
//        largest POSITIVE tau  ==  the ring microphone that arrives EARLIEST
//        every tau <= 0       ==  no ring microphone is early, i.e. the
//                                CENTRE microphone (index 6) heard it first
//
//    See the tau-polarity note at the top of doa_solver.v: `lag > 0` means the
//    ring microphone is EARLY.  The opposite convention here would report the
//    microphone that arrives LAST, and would still "look plausible" because
//    the answer is always one of the seven.
//
//  WHY THE RAW TDOA AND NOT THE FILTERED ONE
//    The event has to be judged on the frame where the sound STARTS.  The
//    low-pass in tdoa_filter is a plain IIR (avg += (x-avg)>>SH) that only
//    updates while a peak is valid, so at the instant the energy gate opens
//    its average still holds the PREVIOUS sound event -- possibly minutes old.
//    Latching the filtered value would therefore report whoever was talking
//    last, not whoever is talking now.  The raw per-frame TDOA is noisy
//    (sigma ~ 0.4 sample = 4 degrees) but the six microphones are 60 degrees
//    apart, i.e. ~6 samples, so the ordering it produces is unambiguous.
//
//  ONE REPORT PER SOUND EVENT
//    `shot` is set when the event is latched and is cleared only after
//    snd_now has been low for GAP_MS.  While `shot` is set no further event is
//    reported, so a single utterance produces exactly one line no matter how
//    many solved frames it contains -- and the line names the FIRST microphone
//    of that utterance, never a later one.
//
//  MICROPHONE -> LED NUMBER
//    The three LEDs lit for a sector come from sk9822_ring's rule
//        ring_dist(led_index(w), sector) <= RING_SPAN
//    which lights the LOGICAL indices {s-1, s, s+1} and maps them to physical
//    strip slots through LED_REVERSE.  This module recomputes the same
//    expression from the microphone's own geometric direction, so the reported
//    numbers are the ones that actually light up -- and they follow
//    SECT_OFF_DEG / MIRROR_X / LED_REVERSE automatically instead of being a
//    hand-written table that goes stale the moment the ring is re-calibrated.
//
//    Mic k sits at phi_k = 90 - 60k degrees in the microphone frame (that is
//    the same phi table doa_solver uses).  For a source exactly at phi_k the
//    sector quantiser
//        sector = floor((theta + SECT_OFF_DEG + 180) / 30)
//    gives the sector whose LEDs face that microphone.  With the shipped
//    SECT_OFF_DEG = 45 that is 10 - 2k, i.e. sectors 10, 8, 6, 4, 2, 0, and
//    with LED_REVERSE = 1 the physical centres become 1, 3, 5, 7, 9, 11 --
//    microphone k faces LED 2k+1, which is what the board actually shows.
//
//  The centre microphone (index 6) has no direction, so it reports no LED
//  number; the caller prints "--,--,--" in that case.
//===========================================================================
module first_arrival #(
    parameter integer NMIC         = 6,
    parameter integer FRAC_W       = 10,        // tdoa_engine fractional bits
    parameter integer NSEC         = 12,        // LED sectors on the ring
    parameter integer SECT_OFF_DEG = 0,         // must equal top.v's value
    parameter integer MIRROR_X     = 0,         // must equal doa_solver's value
    parameter integer LED_REVERSE  = 0,         // must equal sk9822_ring's value
    parameter integer GAP_MS       = 600,       // silence that re-arms the event
    parameter integer CLK_HZ       = 50_000_000
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // ---- one pulse per usable TDOA frame ------------------------------
    //   Must be the strobe that goes WITH `lag`/`frac`/`valid`, and `valid`
    //   must be the magnitude-guarded mask (top.v: strobes[2] +
    //   valid_eff_r).  A bare tdoa_done would present the flags one cycle
    //   early and the ordering could be taken from the previous frame.
    input  wire                        frame_done,
    input  wire [4*NMIC-1:0]           lag,
    input  wire [(FRAC_W+1)*NMIC-1:0]  frac,
    input  wire [NMIC-1:0]             valid,

    // ---- "somebody is talking" ----------------------------------------
    input  wire                        snd_now,

    // ---- event output -------------------------------------------------
    output reg                         ev_stb,     // 1 clk pulse, once per event
    output reg  [2:0]                  ev_mic,     // 0..5 ring, 6 = centre first
    output reg  signed [15:0]          ev_tau,     // that mic's tau, Q8 samples
    output reg  [3:0]                  ev_l,       // left  LED, physical number
    output reg  [3:0]                  ev_c,       // centre LED
    output reg  [3:0]                  ev_r,       // right LED
    output reg  [3:0]                  ev_sect,    // that mic's nominal sector
    output reg                         ev_held     // an event is latched/blocked
);

    //==================================================================
    // 1.  RAW TDOA PER MICROPHONE, Q8 SAMPLES
    //
    //     Same arithmetic as top.v's tau8: (lag << FRAC_W) + frac, then
    //     rounded from Q(FRAC_W) to Q8.  Built from the RAW engine outputs,
    //     not from the filter -- see the header.
    //
    //     REGISTERED BEFORE THE ARGMAX, and that register is not decoration.
    //     Measured with it missing (build of 2026-11-01 11:25):
    //         u_tdoa/frac_flat -> (tau8 arithmetic) -> 5-level argmax
    //     came out as ONE 24.49 ns cone, the worst SETUP path in the design at
    //     -0.396 ns.  The synthesiser had also merged the identical
    //     expression inside tdoa_filter into the same cone, dragging
    //     u_tdoa_lpf/g_lpf[*].x_now into it.  One register splits it into two
    //     short cones; the event report is ~84 ms apart, so a clock of extra
    //     latency is free.
    //==================================================================
    wire [16*NMIC-1:0] tau8_flat;
    reg  [16*NMIC-1:0] tau8_r;          // argmax input, one cycle behind

    genvar gi;
    generate
        for (gi = 0; gi < NMIC; gi = gi + 1) begin : g_raw
            wire signed [3:0]      lg = $signed(lag[4*gi +: 4]);
            wire signed [FRAC_W:0] fr = $signed(frac[(FRAC_W+1)*gi +: (FRAC_W+1)]);
            wire signed [31:0]     tq = ({{28{lg[3]}}, lg} <<< FRAC_W)
                                      + {{(31-FRAC_W){fr[FRAC_W]}}, fr};
            // Q(FRAC_W) -> Q8, round to nearest
            wire signed [31:0]    tq8 = (tq + (32'sd1 <<< (FRAC_W-9))) >>> (FRAC_W-8);

            // |tau| <= R/c = 5.7 samples, so the clamp never bites; it exists
            // to make the 32 -> 16 bit narrowing explicit (same reasoning as
            // top.v: it silences WARN EX3791 rather than hiding a real bug).
            assign tau8_flat[16*gi +: 16] = (tq8 >  32'sd32767) ? 16'sh7FFF :
                                            (tq8 < -32'sd32768) ? 16'sh8000 :
                                                                  tq8[15:0];
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tau8_r <= {16*NMIC{1'b0}};
        else        tau8_r <= tau8_flat;
    end

    // `frame_d1` / `frame_d2` are the strobes that go with the two pipeline
    // stages inside this module.  Contents of tau8_r come from frame N and
    // are visible during cycle N+1; the level-1 winners are visible during
    // N+2, which is when mic_sel/tau_sel first describe frame N.  So the
    // event is judged on frame_d2 -- with frame_d1 the latch would sample the
    // PREVIOUS frame's answer and nothing would look wrong except the number.
    // `valid` needs no delay: it is a register in top.v and holds until the
    // next frame.
    reg frame_d1, frame_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_d1 <= 1'b0;
            frame_d2 <= 1'b0;
        end else begin
            frame_d1 <= frame_done;
            frame_d2 <= frame_d1;
        end
    end

    //==================================================================
    // 2.  ARGMAX OVER THE SIX RING MICROPHONES
    //
    //     Explicit comparison tree, NOT a for loop with a variable index:
    //     indexing a vector with a loop variable makes the synthesiser build
    //     a barrel shifter (measured on the same construct in top.v: several
    //     hundred extra LUTs and a much worse floorplan).
    //
    //     A microphone whose peak was rejected by the magnitude guard is
    //     given -infinity so it can never win.  "valid" is part of the
    //     ordering on purpose: a channel that produced NO peak has tau = 0,
    //     which would otherwise tie with the centre microphone and could be
    //     reported as "first" purely because of its index.
    //
    //     Ties keep the LOWER index (each node is `a > b ? a : b` with `a`
    //     covering the higher indices), so the answer does not depend on
    //     synthesis ordering.
    //
    //     WIDTH AND DEPTH ARE BOTH DELIBERATE.  A first version compared the
    //     full 16-bit tau through five cascaded levels in one cone and PnR
    //     measured 24.85 ns on it (-0.311 ns) with ~2.2 ns per level, i.e.
    //     mostly ROUTING between scattered LUTs.  Two changes bring it down:
    //       * 12 significant bits instead of 16.  |tau| <= R/c = 5.7 samples
    //         = 1459 in Q8, so 12 signed bits (+-2047) hold every physically
    //         possible value with 40% headroom.  Comparing leading zero /
    //         sign-extension bits costs LUTs and can never change the answer.
    //       * the three level-1 winners are REGISTERED, splitting five levels
    //         into 1 + 2 + 2.
    //     Nothing here is on a real-time path: an event is reported once per
    //     utterance, so three cycles of latency cost nothing.
    //==================================================================
    wire signed [11:0] t0 = $signed(tau8_r[11:0]);
    wire signed [11:0] t1 = $signed(tau8_r[27:16]);
    wire signed [11:0] t2 = $signed(tau8_r[43:32]);
    wire signed [11:0] t3 = $signed(tau8_r[59:48]);
    wire signed [11:0] t4 = $signed(tau8_r[75:64]);
    wire signed [11:0] t5 = $signed(tau8_r[91:80]);

    localparam signed [12:0] NEG_INF = 13'sh1000;   // -4096, below any real tau

    wire signed [12:0] e0 = valid[0] ? {t0[11], t0} : NEG_INF;
    wire signed [12:0] e1 = valid[1] ? {t1[11], t1} : NEG_INF;
    wire signed [12:0] e2 = valid[2] ? {t2[11], t2} : NEG_INF;
    wire signed [12:0] e3 = valid[3] ? {t3[11], t3} : NEG_INF;
    wire signed [12:0] e4 = valid[4] ? {t4[11], t4} : NEG_INF;
    wire signed [12:0] e5 = valid[5] ? {t5[11], t5} : NEG_INF;

    // ---- level 1: three pairs, REGISTERED ------------------------------
    reg signed [12:0]  m10_r, m11_r, m12_r;
    reg  [2:0]         i10_r, i11_r, i12_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m10_r <= 13'sd0; m11_r <= 13'sd0; m12_r <= 13'sd0;
            i10_r <= 3'd0;   i11_r <= 3'd2;   i12_r <= 3'd4;
        end else begin
            m10_r <= (e1 > e0) ? e1 : e0;
            i10_r <= (e1 > e0) ? 3'd1 : 3'd0;

            m11_r <= (e3 > e2) ? e3 : e2;
            i11_r <= (e3 > e2) ? 3'd3 : 3'd2;

            m12_r <= (e5 > e4) ? e5 : e4;
            i12_r <= (e5 > e4) ? 3'd5 : 3'd4;
        end
    end

    // ---- level 2 -------------------------------------------------------
    wire               g20 = (m11_r > m10_r);
    wire signed [12:0] m20 = g20 ? m11_r : m10_r;
    wire [2:0]         i20 = g20 ? i11_r : i10_r;

    // ---- level 3 -------------------------------------------------------
    wire               g30 = (m12_r > m20);
    wire signed [12:0] mx  = g30 ? m12_r : m20;
    wire [2:0]         ix  = g30 ? i12_r : i20;

    wire any_v = |valid;

    // No ring microphone early -> the CENTRE one (index 6) is first.
    // 13-bit mx sign-extended back to the 16-bit ev_tau field.
    wire [2:0]         mic_sel = (mx > 13'sd0) ? ix : 3'd6;
    wire signed [15:0] tau_sel = (mx > 13'sd0)
                               ? {{3{mx[12]}}, mx}
                               : 16'sd0;

    //==================================================================
    // 3.  MICROPHONE -> NOMINAL SECTOR, AS AN ELABORATION-TIME TABLE
    //
    //     Evaluated at elaboration time, so this costs no logic beyond the
    //     6-way mux in section 5.  Every constant is derived from the same
    //     expressions doa_solver.v uses, which is the point: re-calibrating
    //     SECT_OFF_DEG or flipping MIRROR_X/LED_REVERSE moves the numbers
    //     here at the same time, with no table to forget.
    //==================================================================
    localparam integer OFF_Q = (SECT_OFF_DEG * 65536) / 180;

    // 7 entries: mic 0..5 plus a placeholder for the centre microphone.
    wire [4*(NMIC+1)-1:0] sec_flat;

    generate
        for (gi = 0; gi < NMIC; gi = gi + 1) begin : g_sec
            // Mic k sits at 90 - 60k degrees.  With MIRROR_X the solver's
            // bearing is negated, so the direction that lights mic k's sector
            // is -(90-60k) = 60k-90.
            localparam integer PHI_DEG = MIRROR_X ? (60*gi - 90) : (90 - 60*gi);
            // 1 degree = 65536/180 = 364.09 LSB; integer division matches
            // doa_solver's SECT_OFF_Q so both round the same way.
            localparam integer PHI_Q0  = (PHI_DEG * 65536) / 180;
            // Fold into the atan2 range.  phi_5 = -210 deg becomes +150 deg,
            // which is the same direction but would otherwise overflow the
            // quantiser's domain.
            localparam integer PHI_Q   = (PHI_Q0 >=  65536) ? PHI_Q0 - 131072 :
                                         (PHI_Q0 <  -65536) ? PHI_Q0 + 131072 : PHI_Q0;
            localparam integer CZ0     = PHI_Q + OFF_Q;
            localparam integer CZ      = (CZ0 >=  65536) ? CZ0 - 131072 :
                                         (CZ0 <  -65536) ? CZ0 + 131072 : CZ0;
            // floor((theta + SECT_OFF + pi) / 2pi * NSEC), the same quantiser
            // as doa_solver.v.  CZ + 65536 is in 0..131072 so the division is
            // an arithmetic shift and cannot disagree with the RTL.
            localparam integer SEC_RAW = ((CZ + 65536) * NSEC) / 131072;
            localparam integer SEC     = (SEC_RAW >= NSEC) ? NSEC - 1 :
                                         (SEC_RAW <  0)    ? 0 : SEC_RAW;

            assign sec_flat[4*gi +: 4] = SEC[3:0];
        end
    endgenerate

    assign sec_flat[4*NMIC +: 4] = 4'd0;      // centre microphone: no direction

    //==================================================================
    // 4.  ONE REPORT PER SOUND EVENT
    //
    //     fire  : a usable frame arrived while the gate is open and this
    //             event has not been reported yet.
    //     shot  : set by fire, cleared only after `snd_now` has been low for
    //             GAP_MS.  This is the "only the first one" guarantee.
    //     gap_cnt is held at 0 while sound is present, so a steady talker can
    //             never re-arm in the middle of a sentence.
    //==================================================================
    localparam integer CLK_PER_MS = (CLK_HZ < 1000) ? 1 : (CLK_HZ / 1000);
    localparam integer GAP_CYC    = (GAP_MS <= 0) ? 1 : (GAP_MS * CLK_PER_MS);
    localparam integer GW         = (GAP_CYC < 2) ? 1 : $clog2(GAP_CYC + 1);

    reg [GW-1:0] gap_cnt;
    reg          shot;

    wire gap_hit = (gap_cnt == GAP_CYC[GW-1:0] - 1'b1);
    wire fire    = snd_now && frame_d2 && any_v && !shot;

    //------------------------------------------------------------------
    // Pipeline stage 1 -> 2 boundary (see section 3): the argmax result is
    // registered so the variable-index sector mux does not sit in the same
    // cone as five cascaded compares.
    //------------------------------------------------------------------
    reg              fire_d;
    reg [2:0]        mic_lat;
    reg signed [15:0] tau_lat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fire_d  <= 1'b0;
            mic_lat <= 3'd6;
            tau_lat <= 16'sd0;
        end else begin
            fire_d <= fire;
            if (fire) begin
                mic_lat <= mic_sel;
                tau_lat <= tau_sel;
            end
        end
    end

    //==================================================================
    // 5.  LATENCY-REGISTERED MIC -> LED   (see the note in section 3)
    //
    //     A variable index into `sec_flat` is a 6-way mux; kept behind
    //     mic_lat it is a short cone on its own, not the tail of the argmax.
    //==================================================================
    localparam [3:0] NSEC_LAST = NSEC - 1;   // 4-bit: see the EX3791 note

    wire [3:0] sec_sel = sec_flat[4*mic_lat +: 4];
    wire       no_dir  = (mic_lat == 3'd6);   // centre reference: no bearing

    // sk9822_ring lights the LOGICAL {s-1, s, s+1}; the strip slot w carries
    // logical index (NSEC-1-w) when LED_REVERSE = 1, so the physical centre is
    // NSEC-1-s and its neighbours stay centre+-1 around the loop.  Identical
    // to mic_uart_framer.v's rpt_c/rpt_l/rpt_r -- if those two ever disagree,
    // the UART line names LEDs that are not the ones lighting up.
    wire [3:0] phys_c = LED_REVERSE ? (NSEC_LAST - sec_sel) : sec_sel;
    wire [3:0] phys_l = (phys_c == 4'd0)          ? NSEC_LAST : (phys_c - 4'd1);
    wire [3:0] phys_r = (phys_c == NSEC_LAST)     ? 4'd0      : (phys_c + 4'd1);

    // 15 is "no LED": the centre microphone has no direction, so there are no
    // three LEDs facing it.  A zero here would be indistinguishable from LED 0.
    wire [3:0] led_l = no_dir ? 4'hF : phys_l;
    wire [3:0] led_c = no_dir ? 4'hF : phys_c;
    wire [3:0] led_r = no_dir ? 4'hF : phys_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            gap_cnt <= {GW{1'b0}};
        else if (snd_now)
            gap_cnt <= {GW{1'b0}};
        else if (!gap_hit)
            gap_cnt <= gap_cnt + 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            shot <= 1'b0;
        else if (!snd_now && gap_hit)
            shot <= 1'b0;
        else if (fire)
            shot <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ev_stb  <= 1'b0;
            ev_mic  <= 3'd6;
            ev_tau  <= 16'sd0;
            ev_l    <= 4'hF;
            ev_c    <= 4'hF;
            ev_r    <= 4'hF;
            ev_sect <= 4'd0;
            ev_held <= 1'b0;
        end else begin
            ev_stb  <= 1'b0;          // single-cycle pulse, one per event
            ev_held <= shot;
            if (fire_d) begin
                ev_stb  <= 1'b1;
                ev_mic  <= mic_lat;
                ev_tau  <= tau_lat;
                ev_sect <= sec_sel;
                ev_l    <= led_l;
                ev_c    <= led_c;
                ev_r    <= led_r;
            end
        end
    end

endmodule
