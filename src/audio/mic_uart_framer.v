//-----------------------------------------------------------------------------
// mic_uart_framer.v
//   Packs the 7 captured channels into a byte stream and feeds uart_tx.
//
//   PACKET FORMAT -- 26 bytes, MSB FIRST (this was documented as "20 bytes,
//   little-endian" for a long time; both halves of that were wrong, which is
//   the kind of mismatch that costs an afternoon on the PC side)
//     byte 0  : 0xA5              sync
//     byte 1  : 0x5A              sync
//     byte 2  : frame_cnt[7:0]    rolling frame counter
//     byte 3  : 0x07              channel count = 7
//     byte 4..6   : ch0[23:16], ch0[15:8], ch0[7:0]
//     byte 7..9   : ch1 ...
//     ...
//     byte 22..24 : ch6 ...
//     byte 25 : XOR checksum of bytes 2..24
//
//   Each 24-bit sample is sent as 3 bytes, MOST significant byte first.  Only
//   the low 24 bits of the 32-bit internal value are sent -- enough for the
//   I2S data width.
//
//   PKT_MODE = 1 selects the DOA diagnostic packet instead (sync A5 5B):
//     byte 0  : 0xA5              byte 1  : 0x5B
//     byte 2  : frame_cnt[7:0]    byte 3  : 0x06   microphone count
//     bytes 4..15  : tau0..tau5, signed Q8 samples (1/256 of a sample),
//                    big-endian.  One sample is 10.2 deg of bearing at
//                    R=40mm / 48.8kHz, so 1 LSB = 0.04 deg.
//     bytes 16..17 : bearing, signed Q1.14 (16384 == pi/2 == 90 deg)
//     byte 18 : quality, 0..255        byte 19 : sector, 0..NSEC-1
//     byte 20 : largest fit residual, Q8 samples (0..255 = 0..1 sample)
//     byte 21 : per-microphone valid mask, bit m = mic m had a real peak
//     bytes 22..24 : per-microphone PEAK LEVEL codes, 3 bits per mic, mic0
//                    in the low bits, top 6 bits zero.  Each step is one
//                    halving (6 dB) below the strongest microphone's peak:
//                    0 = strongest, 7 = more than 42 dB down.  This is the
//                    "which microphone is broken" column -- read it first
//                    when the bearing is stubbornly wrong.
//     byte 25 : XOR checksum of bytes 2..24
//
//   PKT_MODE = 2 prints human-readable text instead -- see the block further
//   down ("HUMAN-READABLE LINES").  In short, one line per displayed direction:
//
//       ANG=180deg  LED=05,06,07  M=3  Q=200
//
//   which is the angle in degrees, the three physical lamp numbers, the
//   microphone that heard the wave first, and the fit quality -- at the rate
//   the lamp ring changes (DIR_MIN_MS = 500 ms), not at the solve rate.
//
//   One diagnostic packet is emitted per SOLVED bearing (~187 Hz), which the
//   115200 baud link carries without dropping anything -- check `dropped`.
//
//   TRANSMIT RATE: the framer waits for ch_stb (one per array frame), then
//   pushes the whole packet.  A 'drop' counter records frames that arrived
//   while the previous packet was still going out, so you can see on the PC
//   whether the UART is fast enough for the array's frame rate.
//
//   IT IS NOT FAST ENOUGH TO BE A RECORDER:
//     26 bytes/packet at 115200 8N1 = 11520 B/s = 443 packets/s
//     array frame rate (BCK/64, ~3.1 MHz BCK) ~= 48 kHz
//   so roughly 1 frame in 108 is actually transmitted.  Treat the UART as a
//   spot-check channel for the raw waveform; the DOA path consumes every
//   frame independently.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module mic_uart_framer #(
    parameter integer DATA_W  = 24,
    // 0 = raw channel waveform (26 bytes, sync A5 5A) -- what the existing PC
    //     tool expects.  The array runs at ~48.8 kHz against a 115200 baud
    //     link, so most frames are dropped and `dropped` says so.
    // 1 = DOA diagnostic packet (26 bytes, sync A5 5B) -- one packet per
    //     SOLVED bearing (~187 Hz), which fits the link with no drops at all.
    //     Use this mode to calibrate the array and to tell a microphone
    //     problem from an algorithm problem: it carries every per-mic TDOA,
    //     the fit residual and the per-mic valid mask.  Layout in the file
    //     header.
    // 2 = human-readable ASCII, one line per DISPLAYED direction (0.5 s, paced
    //     by the ring's own refresh strobe), plus one "A" line per sound
    //     event.  Any terminal at 115200:
    //         ANG=180deg  LED=05,06,07  M=3  Q=200
    //         NO DIRECTION  N=1A2B  D=7F  E=..  W=3      (when nothing shows)
    //     This is the mode to use when a person rather than a script is
    //     reading the output; every number behind it is in mode 1.
    parameter integer PKT_MODE = 0,
    // Silence watchdog for PKT_MODE = 2, in core clocks (~0.52 s at 50 MHz).
    // A parameter, not a localparam, purely so tb_uart_nodir can shrink it.
    parameter integer SIL_CYC  = 26_000_000,

    // ---- OUTPUT POLICY (2026-11-01) -----------------------------------
    //  Board request: nothing on the wire unless a lamp is actually lit.
    //
    //  SILENT_WHEN_DARK = 1 suppresses the "NO DIRECTION ..." status line.
    //  It used to repeat every ~0.52 s forever, which is most of the traffic
    //  on a bench where the array sits in a quiet room -- exactly the noise
    //  this request is about.  Set it to 0 to get the old behaviour back; the
    //  line is still the FIRST thing to ask for when the chain goes dead,
    //  because it is the only output that appears when nothing ever solves.
    //
    //  EV_ONLY_WHEN_LIT = 1 gates the "A" first-arrival line on the ring
    //  really being lit (its G= field reads 1).  The A event fires when the
    //  sound STARTS, which is usually before the sector vote has committed
    //  (~3 frames = 250 ms), so with this on the A line only shows up when
    //  the ring was already lit by the previous phrase: continuous talking
    //  keeps it, the first isolated word does not.  Set to 0 for the old
    //  always-report behaviour while bringing a link up.
    parameter integer SILENT_WHEN_DARK  = 1,
    parameter integer EV_ONLY_WHEN_LIT  = 1,
    // Must match top.v's LED_REVERSE: it selects whether the "L=" field of the
    // periodic report names logical or physical LED numbers (see rpt_c below).
    // Defaulted to the production value so an instance that forgets to pass it
    // cannot silently report three LEDs that are not the three that light up.
    parameter         LED_REVERSE = 1'b1
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        ch_stb,
    input  wire signed [DATA_W-1:0] ch0,
    input  wire signed [DATA_W-1:0] ch1,
    input  wire signed [DATA_W-1:0] ch2,
    input  wire signed [DATA_W-1:0] ch3,
    input  wire signed [DATA_W-1:0] ch4,
    input  wire signed [DATA_W-1:0] ch5,
    input  wire signed [DATA_W-1:0] ch6,
    input  wire [15:0] frame_cnt,

    // ---- raw channel health (see top.v section 3b) ---------------------
    //   ch_act[k] : channel k's raw sample keeps moving (0 = stuck / dead)
    //   ch_lvl[k] : log2 of the running mean |sample|, an absolute 6 dB step
    // These drive the periodic STATUS LINE, which is emitted whether or not
    // anything is ever solved -- that is the whole point: when the chain
    // produces no solution at all, this is the only output left.
    input  wire [6:0]  ch_act,
    input  wire [34:0] ch_lvl,

    // ---- DOA diagnostics (only read when PKT_MODE = 1) ----
    input  wire                     diag_stb,   // 1-clk: new bearing published
    input  wire [(16*6)-1:0]        diag_tau,   // 6 x signed Q8 samples
    input  wire signed [15:0]       diag_bear,  // Q1.14, 16384 == pi/2
    input  wire [7:0]               diag_qual,  // 0..255 fit quality
    input  wire [3:0]               diag_sect,  // 0..NSEC-1
    input  wire [7:0]               diag_resid, // Q8 samples, largest residual
    input  wire [5:0]               diag_valid, // per-microphone valid mask
    input  wire [(3*6)-1:0]         diag_lvl,   // per-mic peak level, 6 dB steps

    // ---- engine-side reference energy (the gate's own number) -----------
    //   log2(ref_energy), saturated to 15.
    //
    //   This is the quantity the "is anyone talking" gate (snd_now in top.v)
    //   actually compares against ANY_SND_E, so it is the ONLY field that can
    //   answer "why is leds[2] dark?".
    //
    //   It is deliberately NOT the same as the E= field above.  ch_lvl
    //   measures the RAW sample, while ref_energy is computed on the
    //   mic_cond OUTPUT -- and between them sits a DC block, the pre-emphasis
    //   (0.5 - 0.46875 z^-1) and a low-pass, which together cost about 10 dB
    //   on white noise and 20..30 dB across speech (that chain is a high-pass
    //   with its corner near 500 Hz).  A board can therefore read a healthy
    //   E=17 while the engine sees almost nothing: 17 in raw terms is
    //   mean|sample| = 2^17, but if that energy is low-frequency it is
    //   exactly what the chain is designed to remove.
    //
    //   Reading it:  W=<hex>, 2^W == ref_energy.
    //       W = 0..4   effectively no signal at the engine
    //       W = 6      ~64      (half the ORIGINAL 1024 gate)
    //       W = 7      ~128     <- the current ANY_SND_E, in these units
    //       W = 10     ~1024    (the gate this firmware used before 10-16)
    //       W = 15     >=32768  (loud)
    //   Compare it between silence and speech: that single pair sets
    //   ANY_SND_E correctly, with no guesswork.
    input  wire [3:0]               diag_energy,

    // ---- bearing in whole degrees, 0..359 -------------------------------
    //   The number a mounting-rotation calibration actually needs.  S= gives
    //   the 30 degree bin the ring is showing; B= gives the angle behind it,
    //   so an offset can be measured instead of guessed at.
    //   Calibration: point a source at a known angle, compare B= with it, and
    //   put the difference into SECT_OFF_DEG in top.v.
    input  wire [8:0]               diag_deg,

    // ---- the line that mirrors the lamp ring (PKT_MODE = 2 only) --------
    //   Emitted ONCE PER DISPLAYED DIRECTION, on the ring's own refresh
    //   strobe (top.v dir_update, throttled to DIR_MIN_MS = 500 ms), so the
    //   serial log and the ring change at the same instant and at the same
    //   rate.  Nothing is printed at the ~187 Hz solve rate.
    //
    //   rpt_mic  : 0..5 = the ring microphone the displayed group is centred
    //              on (2026-11-01: derived from rpt_sect in top.v, so M= and
    //              the LED numbers on the same line can never disagree -- it
    //              used to be a raw-TDOA argmax and did disagree).  The value
    //              6 ("the centre mic heard it first") is no longer produced.
    //   rpt_sect : the sector the ring is currently showing (top.v hands over
    //              sect_q -- that sector re-centred on the mic axis -- i.e.
    //              what is on the lamps, not the solver's live output).
    //
    //   rpt_tau is gone: the raw TDOA in Q8 samples was one of the "raw
    //   values" this line exists to replace (and it is still in the mode 1
    //   packet).  rpt_lit stays because the "A" event line prints G=<0|1>,
    //   and an event can be judged on a frame the ring is already blanking.
    input  wire                     rpt_stb,
    input  wire [2:0]               rpt_mic,
    input  wire [3:0]               rpt_sect,
    input  wire                     rpt_lit,

    // ---- first-arrival event (PKT_MODE = 2 only) ------------------------
    //   Emitted ONCE per sound event, on the frame the event was judged, and
    //   blocked until the room has been quiet for first_arrival's GAP_MS.
    //   This is the line to read when the question is "who heard it first",
    //   NOT the periodic F line: F is a 5 s snapshot of whatever the solver
    //   currently believes, so it repeats while nothing changes.
    //
    //   ev_mic  : 0..5 = ring microphone that arrived FIRST,
    //             6    = no ring microphone was early, i.e. the CENTRE
    //                    reference (index 6) heard it first
    //   ev_tau  : that microphone's RAW TDOA, Q8 samples, positive = earlier
    //   ev_l/c/r: the three LEDs that face that microphone, in PHYSICAL strip
    //             order, derived by first_arrival from the same sector
    //             quantiser and LED_REVERSE rule the ring uses.  For ev_mic=6
    //             there is no direction and the line prints "--,--,--".
    input  wire                     ev_stb,
    input  wire [2:0]               ev_mic,
    input  wire signed [15:0]       ev_tau,
    input  wire [3:0]               ev_l,
    input  wire [3:0]               ev_c,
    input  wire [3:0]               ev_r,

    output wire        tx,
    output reg  [15:0] dropped,     // frames skipped because UART was busy
    output reg  [7:0]  bytes_sent   // rolling debug counter
);

    //------------------------------------------------------------------
    // UART
    //------------------------------------------------------------------
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;

    audio_uart_tx #(
        .CLK_HZ (50_000_000),
        .BAUD   (115200)
    ) u_uart (
        .clk     (clk),
        .rst_n   (rst_n),
        .tx_start(tx_start),
        .tx_data (tx_data),
        .tx      (tx),
        .tx_busy (tx_busy)
    );

    //------------------------------------------------------------------
    // Packet buffer: 26 bytes.
    //   [0]=A5 [1]=5A [2]=cnt [3]=0x07
    //   [4..24] = 7 x 3 bytes
    //   [25] = checksum
    //------------------------------------------------------------------
    localparam integer PKT_LEN = 26;

    reg [7:0]  pkt [0:PKT_LEN-1];
    //  7 bits, not 6: the NO DIRECTION line grew to 77 characters on
    //  2026-11-01, and a 6-bit counter wraps at 63 -- the line then stopped
    //  mid-way through the E= field (caught by tb_uart_nodir, not on the
    //  board, where the symptom would have looked like a truncated serial
    //  line and been blamed on the cable).
    reg [6:0]  pkt_idx;        // 7 bits: the ASCII lines are up to 77 characters
    reg [1:0]  state;          // 0 idle, 1 loading, 2 sending
    reg [7:0]  csum;

    integer i;

    //------------------------------------------------------------------
    // HUMAN-READABLE LINES  (PKT_MODE = 2)   -- rewritten 2026-11-01
    //
    //   Replaces the hex dump (Q=CB R=14 V=3F S=07 L=0A55C / T=0000,FB12,...)
    //   and the 84 ms status line.  What is printed now is what a person
    //   reads off the board -- an angle in degrees and the lamp numbers --
    //   at the rate the lamps change, 2 Hz, and nothing else:
    //
    //   ANG=180deg  LED=05,06,07  M=3  Q=200
    //
    //   ANG : bearing in DECIMAL DEGREES, 0..359 -- ANG = theta + 180
    //         (mod 360), the same angle the old B= field carried.  The +180
    //         is the offset inside doa_solver's sector quantiser, kept
    //         because the mounting calibration is defined on it:
    //             SECT_OFF_DEG = wrap180(true_angle - ANG)
    //   LED : the three lamps that are lit, in PHYSICAL strip order 0..11
    //         (LED_REVERSE already applied, exactly like the "A" line), so
    //         these are the numbers you can count on the ring itself.
    //   M   : which microphone heard the wave FIRST -- 0..5 = ring, 6 = the
    //         centre reference.  Kept from the old F line because it is the
    //         one thing mode 1 cannot be read for at a glance.
    //   Q   : fit quality 0..255, decimal, larger is better.  Says whether to
    //         trust ANG at all.
    //
    //   and, whenever 0.52 s passes with NO direction displayed (silence, or
    //   nothing ever solved), one of these instead, so the link never goes
    //   silent and "why is nothing shown" can be diagnosed -- this line is the
    //   only instrument left once the status lamps are removed:
    //
    //   NO DIRECTION N=1A2B D=7F W=3 Q=060 R=14 V=3F L=012345 S=0B E=090A0A090B0B09
    //
    //   L : per-microphone CORRELATION PEAK LEVEL, mic0 first, one decimal digit
    //       each, in 6 dB steps below the strongest microphone.  A microphone
    //       that is quiet, blocked or mis-wired sits at 4..6; one that produced
    //       no usable peak at all saturates at 7 AND has its V bit clear.  This
    //       is the field that says whether an absent V bit means "no signal on
    //       that channel" or "the magnitude guard rejected it".
    //   S : the sector the SOLVER produced on its last pass, whether or not it
    //       was good enough to display.  At Q = 0 it still shows whether the
    //       fit is wandering over the whole circle or parked in one sector.
    //
    //   W : log2(ref_energy) -- the number the ENERGY gate compares against
    //       ANY_SND_E.  W below log2(ANY_SND_E) means the engine never even
    //       starts, and no threshold in the solver matters.
    //   Q : the quality (0..255) of the LAST solve that ran, DECIMAL, to be
    //       compared with CONF_MIN.  W high but Q low is the "the
    //       microphones disagree" case, and it is a different repair.
    //   R : largest residual among the surviving microphones, Q8 samples.
    //   V : per-microphone valid mask (bit m = mic m produced a real peak).
    //   N : the audio frame counter.  Frozen means no I2S frames at all.
    //   D : which RAW channels are still moving (bit k = channel k).  D=00
    //       means no microphone is delivering data -- wiring, power or the
    //       bit clock, never a quality question.
    //   E : the raw level of each channel, 2 hex per channel, channel 6
    //       (the CENTRE REFERENCE) LAST, as log2(mean|x|) so every step is
    //       6 dB.  "00" is flat zero, ~09 is a live MEMS noise floor, 17+
    //       is loud.  If the first six are alive and the last one is 00,
    //       the reference channel is the broken one -- and a dead reference
    //       stops every correlation, so no solution ever appears while
    //       led0/led1 still look healthy.
    //   W : log2(ref_energy), the number the "is anyone talking" gate
    //       (ANY_SND_E) actually compares against.
    //   Those four stay hexadecimal: they are the "why is the ring dark"
    //   answer, printed only when there is nothing human-readable to show.
    //
    //   Line 1 is 38 bytes, line 2 is 51; two lines per second cost about
    //   0.8 % of the 115200 link, versus ~30 % for the old 84 ms status line
    //   plus the Q/T line.  The numbers those lines carried are all still in
    //   the PKT_MODE = 1 binary packet, which tools/doa_diag.py decodes --
    //   that mode is untouched.
    //
    //   The decimal digits are recovered with comparator chains (DH9 / TENS9
    //   below) rather than a divider, which keeps the line builder single
    //   cycle and combinational like the hex formatter it replaces.
    //------------------------------------------------------------------
    // Longest line is the NO DIRECTION one: indices 0..76 (75 characters plus
    // CR LF).  Sized to the last index actually written, so no entry of txt[]
    // is left undriven -- GowinSynthesis reports that as EX1998 "net does not
    // have a driver", which is noise the next person has to read and dismiss.
    localparam integer TXT_LEN = 77;

    //------------------------------------------------------------------
    // Silence watchdog.
    //
    // Counts CORE CLOCK cycles, not audio frames: on a board where the I2S
    // side is dead there are no ch_stb pulses at all, and that is exactly
    // when a line saying so is worth most.  Any direction displayed restarts
    // it (rpt_stb), so the serial output settles at exactly one line per half
    // second whether or not a direction is being shown.
    //
    // SIL_CYC is deliberately a hair LONGER than DIR_MIN_MS (500 ms =
    // 25_000_000 cycles): if it were equal, the watchdog could fire on the
    // very cycle the ring was about to refresh and a steady source would
    // alternate ANG= lines with spurious "NO DIRECTION" ones.
    //------------------------------------------------------------------
    // (SIL_CYC is a module parameter -- see the parameter list -- only so a
    //  testbench can shrink it; top.v never overrides it.)

    reg  [24:0] sil_cnt;
    wire        sil_due = (sil_cnt == SIL_CYC-1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sil_cnt <= 25'd0;
        else if (rpt_stb || sil_due)
            sil_cnt <= 25'd0;
        else
            sil_cnt <= sil_cnt + 1'b1;
    end

    reg [7:0] txt [0:TXT_LEN-1];
    // send_len is the number of characters of txt[] to transmit in the ASCII
    // modes; it is written when the line is built and read as the state-3
    // exit condition.
    reg [6:0] send_len;

    function [7:0] hexc;
        input [3:0] v;
        begin
            hexc = (v < 4'd10) ? (8'h30 + {4'b0, v})
                               : (8'h41 + {4'b0, v} - 8'd10);
        end
    endfunction

    //------------------------------------------------------------------
    // DECIMAL DIGITS WITHOUT A DIVIDER
    //
    // dh9 / tens9 recover the hundreds and tens digits of an 0..359 angle,
    // and the hundreds/tens of the 0..255 quality, with comparator chains and
    // one subtract each.  Worst case ~12 comparators per field -- free on this
    // device next to a divider, and it keeps the whole line builder in one
    // clock like the hex formatter it replaces.
    //------------------------------------------------------------------
    function [3:0] dh9;                 // hundreds digit of 0..359
        input [8:0] v;
        dh9 = (v >= 9'd300) ? 4'd3 :
              (v >= 9'd200) ? 4'd2 :
              (v >= 9'd100) ? 4'd1 : 4'd0;
    endfunction

    function [3:0] tens9;               // tens digit of a 0..99 remainder
        input [7:0] v;
        tens9 = (v >= 8'd90) ? 4'd9 : (v >= 8'd80) ? 4'd8 :
                (v >= 8'd70) ? 4'd7 : (v >= 8'd60) ? 4'd6 :
                (v >= 8'd50) ? 4'd5 : (v >= 8'd40) ? 4'd4 :
                (v >= 8'd30) ? 4'd3 : (v >= 8'd20) ? 4'd2 :
                (v >= 8'd10) ? 4'd1 : 4'd0;
    endfunction

    // ---- ANG: bearing 0..359 -> three digits ---------------------------
    wire [3:0] deg_h  = dh9(diag_deg);
    wire [8:0] deg_hv = (deg_h == 4'd3) ? 9'd300 :
                        (deg_h == 4'd2) ? 9'd200 :
                        (deg_h == 4'd1) ? 9'd100 : 9'd0;
    // The 9-bit form is deliberate: `diag_deg - deg_hv` is a 9-bit expression,
    // and assigning it straight to an 8-bit wire makes GowinSynthesis emit
    // EX3791 "Expression size 9 truncated to fit in target size 8".  Taking
    // the low byte explicitly says the top bit is known to be zero (the
    // remainder is 0..99) instead of leaving it to a silent narrowing.
    wire [8:0] deg_r9 = diag_deg - deg_hv;                  // 0..99
    wire [7:0] deg_r  = deg_r9[7:0];
    wire [3:0] deg_t  = tens9(deg_r);
    wire [7:0] deg_tv = ({4'd0, deg_t} << 3) + ({4'd0, deg_t} << 1);  // *10
    wire [7:0] deg_uv = deg_r - deg_tv;                     // 0..9

    // ---- Q: quality 0..255 -> three digits -----------------------------
    wire [1:0] q_h    = (diag_qual >= 8'd200) ? 2'd2 :
                        (diag_qual >= 8'd100) ? 2'd1 : 2'd0;
    wire [7:0] q_hv   = (q_h == 2'd2) ? 8'd200 :
                        (q_h == 2'd1) ? 8'd100 : 8'd0;
    wire [7:0] q_r    = diag_qual - q_hv;                   // 0..99
    wire [3:0] q_t    = tens9(q_r);
    wire [7:0] q_tv   = ({4'd0, q_t} << 3) + ({4'd0, q_t} << 1);      // *10
    wire [7:0] q_uv   = q_r - q_tv;                         // 0..9

    //------------------------------------------------------------------
    //  Geometry of the "F" report: which LEDs the ring lights for a sector
    //
    //  sk9822_ring lights (sector-1, sector, sector+1) modulo NLED, because
    //  ring_dist() is the shortest distance around the loop.  Recomputed here
    //  so the framer can print the numbers rather than three more ports.
    //  NSEC_LAST must follow NSEC in top.v / sk9822_ring.
    //------------------------------------------------------------------
    localparam [3:0] NSEC_LAST = 4'd11;

    // The centre LED of the lit group, in PHYSICAL strip order.
    //
    // sk9822_ring lights the three LOGICAL indices {s-1, s, s+1}.  With
    // LED_REVERSE = 1 the strip slot w carries logical index 11-w, so the
    // physical centre becomes 11-s (and its neighbours 10-s / 12-s, i.e.
    // still centre+-1 around the loop).  Reporting the logical index here
    // would name three LEDs that are not the three that light up -- which is
    // exactly what this field exists to prevent.
    //
    // Must be kept equal to top.v's LED_REVERSE.
    wire [3:0] rpt_c = LED_REVERSE ? (NSEC_LAST - rpt_sect) : rpt_sect;
    wire [3:0] rpt_l = (rpt_c == 4'd0)       ? NSEC_LAST : (rpt_c - 4'd1);
    wire [3:0] rpt_r = (rpt_c == NSEC_LAST)  ? 4'd0      : (rpt_c + 4'd1);

    //------------------------------------------------------------------
    //  REPORT REQUEST LATCH
    //
    //  rpt_stb is a single-clock pulse, but this link is shared with the
    //  status line and the Q/T line: at 115200 one line takes 3..5 ms out of
    //  every ~84 ms, so a bare pulse lands while tx_busy is high and is thrown
    //  away by the state-0 guard (`if (tx_busy || tx_start) dropped++`).
    //  A "once every 5 s" report that silently goes missing is worse than no
    //  report at all, so the request is latched and served as soon as the
    //  transmitter is free.  Cleared ONLY in the branch that builds the line,
    //  so it can never be dropped.
    //------------------------------------------------------------------
    reg rpt_pend;

    //  First-arrival event latch.  Same reasoning as rpt_pend, but with a
    //  HIGHER priority than it: an event is the answer to "who heard it
    //  first" and cannot be reconstructed later, whereas F is a snapshot that
    //  repeats every 5 s.  It also must never be merged with a later event
    //  from the same utterance, so it is cleared only where the line is built.
    reg ev_pend;

    //  Silence-watchdog latch, lowest priority of the three: it only tells
    //  the reader "nothing is being shown right now", so it must never push
    //  an event or a fresh direction out of the way.
    reg sil_pend;

    // two-digit decimal, zero padded -- LED indices run to 11, and a fixed
    // field width keeps every index at a constant position in the line
    function [7:0] dec_hi;
        input [3:0] v;
        begin
            dec_hi = (v >= 4'd10) ? 8'h31 : 8'h30;
        end
    endfunction

    function [7:0] dec_lo;
        input [3:0] v;
        begin
            dec_lo = (v >= 4'd10) ? (8'h30 + {4'b0, v} - 8'd10)
                                  : (8'h30 + {4'b0, v});
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= 2'd0;
            pkt_idx    <= 7'd0;
            tx_start   <= 1'b0;
            tx_data    <= 8'h00;
            dropped    <= 16'd0;
            bytes_sent <= 8'd0;
            csum       <= 8'h00;
            // send_len is the state-3 exit condition ("pkt_idx ==
            // send_len - 1"), so an X/undriven post-reset value can hang
            // state 3.  The normal path writes it in state 0 first, so the
            // symptom is invisible on the board -- which is exactly why it
            // survived until 2026-10-15.
            send_len   <= 7'd0;
            rpt_pend   <= 1'b0;
            ev_pend    <= 1'b0;
            sil_pend   <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            // latch the periodic report request (see REPORT REQUEST LATCH)
            if ((PKT_MODE == 2) && rpt_stb) rpt_pend <= 1'b1;
            // ... and the first-arrival event.  The event wins the arbiter
            // below, so it is never queued behind a 5 s snapshot.
            //  Gated on the ring being lit when EV_ONLY_WHEN_LIT = 1 -- the
            //  board asked for no output at all while nothing is displayed.
            if ((PKT_MODE == 2) && ev_stb &&
                (!EV_ONLY_WHEN_LIT || rpt_lit)) ev_pend <= 1'b1;
            // ... and "half a second with nothing to show" (see SIL_CYC).
            //  Suppressed entirely when SILENT_WHEN_DARK = 1: a dark ring now
            //  produces NO traffic, which is the requested behaviour.
            if ((PKT_MODE == 2) && sil_due && !SILENT_WHEN_DARK)
                sil_pend <= 1'b1;

            case (state)
            //----------------------------------------------------------
            2'd0: begin
                // PRIORITY: first-arrival event, then the periodic F report,
                // then the generic diag / status trigger.  Both dedicated
                // branches wait for the transmitter instead of falling into
                // the "UART busy -> dropped" path below.
                if ((PKT_MODE == 2) && ev_pend && !tx_busy && !tx_start) begin
                    //---- FIRST-ARRIVAL EVENT (31 chars + CR LF) ----
                    //   A M=<1 hex> L=<dd>,<dd>,<dd> T=<4 hex> S=<1 hex> G=<0|1>
                    //
                    //   [0]  "A"
                    //   [2]  M=  microphone that heard the sound FIRST,
                    //        0..5 = ring, 6 = the centre reference
                    //   [6]  L=  the three LEDs facing THAT MICROPHONE, in
                    //        physical strip order.  Computed by first_arrival
                    //        from the same quantiser + LED_REVERSE rule the
                    //        ring uses, so these are the numbers that light
                    //        up when the source is in that microphone's
                    //        direction -- NOT the three LEDs currently lit if
                    //        the solver has drifted elsewhere (compare with S).
                    //        "--,--,--" for M=6, which has no direction.
                    //   [17] T   that microphone's RAW TDOA, Q8 samples,
                    //        two's complement, positive = arrived EARLIER
                    //   [24] S   sector the ring is showing right now
                    //        (logical, before LED_REVERSE)
                    //   [28] G   1 = the ring is lit, 0 = blanked
                    txt[0]  = "A"; txt[1] = " ";
                    txt[2]  = "M"; txt[3] = "=";
                    txt[4]  = hexc({1'b0, ev_mic});
                    txt[5]  = " "; txt[6] = "L"; txt[7] = "=";
                    txt[8]  = (ev_mic == 3'd6) ? "-" : dec_hi(ev_l);
                    txt[9]  = (ev_mic == 3'd6) ? "-" : dec_lo(ev_l);
                    txt[10] = ",";
                    txt[11] = (ev_mic == 3'd6) ? "-" : dec_hi(ev_c);
                    txt[12] = (ev_mic == 3'd6) ? "-" : dec_lo(ev_c);
                    txt[13] = ",";
                    txt[14] = (ev_mic == 3'd6) ? "-" : dec_hi(ev_r);
                    txt[15] = (ev_mic == 3'd6) ? "-" : dec_lo(ev_r);
                    txt[16] = " "; txt[17] = "T"; txt[18] = "=";
                    txt[19] = hexc(ev_tau[15:12]);
                    txt[20] = hexc(ev_tau[11:8]);
                    txt[21] = hexc(ev_tau[7:4]);
                    txt[22] = hexc(ev_tau[3:0]);
                    txt[23] = " "; txt[24] = "S"; txt[25] = "=";
                    txt[26] = hexc(rpt_sect);
                    txt[27] = " "; txt[28] = "G"; txt[29] = "=";
                    txt[30] = rpt_lit ? "1" : "0";
                    txt[31] = 8'h0D;    // CR
                    txt[32] = 8'h0A;    // LF
                    send_len <= 7'd33;
                    pkt_idx  <= 7'd0;
                    ev_pend  <= 1'b0;   // served -- cleared only here
                    state    <= 2'd1;
                end else if ((PKT_MODE == 2) && rpt_pend && !tx_busy && !tx_start) begin
                    //---- DIRECTION LINE (36 chars + CR LF) ----
                    // Assembled with blocking assignments and FIXED indices,
                    // so the length never varies:
                    //   [0]  ANG=<3 dec>deg        bearing, 0..359
                    //   [10] LED=<dd>,<dd>,<dd>    the three lamps that light
                    //   [26] M=<1 hex>             first microphone
                    //   [31] Q=<3 dec>             fit quality
                    //   [36] CR   [37] LF
                    txt[0]  = "A"; txt[1]  = "N"; txt[2]  = "G";
                    txt[3]  = "=";
                    txt[4]  = 8'h30 + {4'd0, deg_h};
                    txt[5]  = 8'h30 + {4'd0, deg_t};
                    txt[6]  = 8'h30 + {4'd0, deg_uv[3:0]};
                    txt[7]  = "d"; txt[8]  = "e"; txt[9]  = "g";
                    txt[10] = " "; txt[11] = " ";
                    txt[12] = "L"; txt[13] = "E"; txt[14] = "D";
                    txt[15] = "=";
                    txt[16] = dec_hi(rpt_l);
                    txt[17] = dec_lo(rpt_l);
                    txt[18] = ",";
                    txt[19] = dec_hi(rpt_c);
                    txt[20] = dec_lo(rpt_c);
                    txt[21] = ",";
                    txt[22] = dec_hi(rpt_r);
                    txt[23] = dec_lo(rpt_r);
                    txt[24] = " "; txt[25] = " ";
                    txt[26] = "M"; txt[27] = "=";
                    txt[28] = hexc({1'b0, rpt_mic});
                    txt[29] = " "; txt[30] = " ";
                    txt[31] = "Q"; txt[32] = "=";
                    txt[33] = 8'h30 + {4'd0, q_h};
                    txt[34] = 8'h30 + {4'd0, q_t};
                    txt[35] = 8'h30 + {4'd0, q_uv[3:0]};
                    txt[36] = 8'h0D;    // CR
                    txt[37] = 8'h0A;    // LF
                    send_len <= 7'd38;
                    pkt_idx  <= 7'd0;
                    rpt_pend <= 1'b0;   // served -- cleared only here
                    sil_pend <= 1'b0;   // a direction was just displayed, so
                                        // no "nothing to show" is owed
                    state    <= 2'd1;
                end else if ((PKT_MODE == 2) && sil_pend && !tx_busy && !tx_start) begin
                    //---- NO-DIRECTION LINE (75 chars + CR LF) ----
                    // Printed instead of going silent, and now the ONLY
                    // diagnostic left on a board whose lamps were removed:
                    //   [0]  NO DIRECTION
                    //   [13] N=<4 hex frame_cnt>   frozen -> no I2S frames
                    //   [20] D=<2 hex ch_act>      00 -> no mic delivers data
                    //   [25] W=<1 hex>             log2(ref_energy), the GATE's
                    //                              own number
                    //   [29] Q=<3 dec>             quality of the LAST solve,
                    //                              0..255 -- compare with
                    //                              CONF_MIN to see which gate
                    //                              is holding the ring dark
                    //   [35] R=<2 hex>             largest residual, Q8
                    //   [40] V=<2 hex>             per-mic VALID mask
                    //   [45] L=<1 dec x 6>         CORRELATION PEAK LEVEL per
                    //                              microphone, mic0 first, in
                    //                              6 dB steps below the strongest
                    //                              (0 = strongest, 7 = 36+ dB
                    //                              down).  This is the field that
                    //                              separates the two cases the
                    //                              V= mask cannot: a mic whose
                    //                              peak is merely SMALL (L=4..6,
                    //                              genuinely quiet or blocked)
                    //                              and one that produced NO peak
                    //                              at all (L=7 with V bit clear).
                    //   [54] S=<2 hex>             sector the SOLVER last
                    //                              computed, valid or not.  Even
                    //                              at Q=0 it says whether the fit
                    //                              is wandering or parked.
                    //   [59] E=<2 hex x 7>         raw level per channel,
                    //                              channel 6 LAST
                    //   [75] CR   [76] LF
                    //
                    //   W vs Q is the whole diagnosis:
                    //     W < log2(ANY_SND_E)  -> the ENERGY gate is shut: the
                    //                             engine is not even started, Q
                    //                             is whatever the last start
                    //                             left behind
                    //     W high, Q < CONF_MIN -> the engine ran and the mics
                    //                             DISAGREE: SNR, reverberation,
                    //                             assembly or one bad channel
                    //     Q >= CONF_MIN        -> you should be seeing an ANG
                    //                             line; if not, the sector vote
                    //                             (SECT_VOTES) never settled
                    txt[0]  = "N"; txt[1]  = "O"; txt[2]  = " ";
                    txt[3]  = "D"; txt[4]  = "I"; txt[5]  = "R";
                    txt[6]  = "E"; txt[7]  = "C"; txt[8]  = "T";
                    txt[9]  = "I"; txt[10] = "O"; txt[11] = "N";
                    txt[12] = " ";
                    txt[13] = "N"; txt[14] = "=";
                    txt[15] = hexc(frame_cnt[15:12]);
                    txt[16] = hexc(frame_cnt[11:8]);
                    txt[17] = hexc(frame_cnt[7:4]);
                    txt[18] = hexc(frame_cnt[3:0]);
                    txt[19] = " ";
                    txt[20] = "D"; txt[21] = "=";
                    txt[22] = hexc({3'b0, ch_act[6:4]});
                    txt[23] = hexc(ch_act[3:0]);
                    txt[24] = " ";
                    txt[25] = "W"; txt[26] = "=";
                    txt[27] = hexc(diag_energy);
                    txt[28] = " ";
                    txt[29] = "Q"; txt[30] = "=";
                    txt[31] = 8'h30 + {4'd0, q_h};
                    txt[32] = 8'h30 + {4'd0, q_t};
                    txt[33] = 8'h30 + {4'd0, q_uv[3:0]};
                    txt[34] = " ";
                    txt[35] = "R"; txt[36] = "=";
                    txt[37] = hexc(diag_resid[7:4]);
                    txt[38] = hexc(diag_resid[3:0]);
                    txt[39] = " ";
                    txt[40] = "V"; txt[41] = "=";
                    txt[42] = hexc({2'b0, diag_valid[5:4]});
                    txt[43] = hexc(diag_valid[3:0]);
                    txt[44] = " ";
                    //  L= : six single digits, mic0 first, so they can be
                    //  counted off against the ring without decoding hex.
                    txt[45] = "L"; txt[46] = "=";
                    for (i = 0; i < 6; i = i + 1)
                        txt[47 + i] = 8'h30 + {5'd0, diag_lvl[3*i +: 3]};
                    txt[53] = " ";
                    txt[54] = "S"; txt[55] = "=";
                    txt[56] = "0";
                    txt[57] = hexc(diag_sect);
                    txt[58] = " ";
                    txt[59] = "E"; txt[60] = "=";
                    for (i = 0; i < 7; i = i + 1) begin
                        txt[61 + 2*i] = hexc({3'b0, ch_lvl[5*i + 4]});
                        txt[62 + 2*i] = hexc(ch_lvl[5*i + 3 -: 4]);
                    end
                    txt[75] = 8'h0D;    // CR
                    txt[76] = 8'h0A;    // LF
                    send_len <= 7'd77;
                    pkt_idx  <= 7'd0;
                    sil_pend <= 1'b0;   // served -- cleared only here
                    state    <= 2'd1;
                end else if ((PKT_MODE == 0) ? ch_stb
                                            : ((PKT_MODE == 1) ? diag_stb : 1'b0)) begin
                    // PKT_MODE = 2 produces nothing on the solve strobe any
                    // more: its two lines are the branches above, both paced
                    // by the ring (0.5 s) rather than by the solver (187 Hz).
                    if (tx_busy || tx_start) begin
                        // UART can't keep up -- drop this frame and count it.
                        // (A pending A / ANG / NO DIRECTION request never
                        //  reaches here: those branches above wait for the
                        //  transmitter instead of dropping the line.)
                        dropped <= dropped + 1'b1;
                    end else if (PKT_MODE == 1) begin
                        //----------------------------------------------
                        // DOA DIAGNOSTIC PACKET (sync A5 5B, 26 bytes)
                        //
                        //   byte  0     : 0xA5
                        //   byte  1     : 0x5B          DOA packet marker
                        //   byte  2     : frame_cnt[7:0]
                        //   byte  3     : 0x06          microphone count
                        //   bytes 4..15 : tau0..tau5, signed Q8 samples,
                        //                 big-endian (high byte first).
                        //                 Q8 = 1/256 sample; one sample is
                        //                 10.2 deg of bearing at R=40mm,
                        //                 48.8kHz, so 1 LSB = 0.04 deg.
                        //   bytes 16..17: bearing, signed Q1.14 (16384 = pi/2)
                        //   byte 18     : quality, 0..255 (0 = mics disagree)
                        //   byte 19     : sector, 0..NSEC-1
                        //   byte 20     : largest fit residual, Q8 samples
                        //   byte 21     : valid mask, bit m = microphone m
                        //                 produced a positive correlation peak
                        //   bytes 22..24: reserved, zero
                        //   byte 25     : XOR checksum of bytes 2..24
                        //----------------------------------------------
                        pkt[0] <= 8'hA5;
                        pkt[1] <= 8'h5B;
                        pkt[2] <= frame_cnt[7:0];
                        pkt[3] <= 8'h06;

                        for (i = 0; i < 6; i = i + 1) begin
                            pkt[4 + 2*i] <= diag_tau[16*i + 15 -: 8];
                            pkt[5 + 2*i] <= diag_tau[16*i +  7 -: 8];
                        end

                        pkt[16] <= diag_bear[15:8];
                        pkt[17] <= diag_bear[7:0];
                        pkt[18] <= diag_qual;
                        pkt[19] <= {4'b0, diag_sect};
                        pkt[20] <= diag_resid;
                        pkt[21] <= {2'b0, diag_valid};
                        // bytes 22..24: the six 3-bit peak-level codes packed
                        // as one 18-bit field, mic0 in the lowest three bits
                        // of byte 24 (see the file header).
                        pkt[22] <= {6'b0, diag_lvl[17:16]};
                        pkt[23] <= diag_lvl[15:8];
                        pkt[24] <= diag_lvl[7:0];

                        csum = 8'h00;
                        csum = csum ^ frame_cnt[7:0];
                        csum = csum ^ 8'h06;
                        for (i = 0; i < 6; i = i + 1) begin
                            csum = csum ^ diag_tau[16*i + 15 -: 8];
                            csum = csum ^ diag_tau[16*i +  7 -: 8];
                        end
                        csum = csum ^ diag_bear[15:8] ^ diag_bear[7:0];
                        csum = csum ^ diag_qual;
                        csum = csum ^ {4'b0, diag_sect};
                        csum = csum ^ diag_resid;
                        csum = csum ^ {2'b0, diag_valid};
                        // bytes 22..24 (the 18 bits of per-mic level codes)
                        // were MISSING here until 2026-10-15.  The header
                        // above, and tools/doa_diag.py
                        //     for b in p[2:25]: csum ^= b
                        //     if csum != p[25]: continue      <-- silently DROPS
                        // both define the checksum as covering bytes 2..24, so
                        // the PC tool rejected EVERY PKT_MODE = 1 packet and
                        // the whole DOA capture / calibration path was dead.
                        // Nothing on the FPGA side noticed, because the field
                        // was simply never covered.
                        // (The raw-waveform branch below already covers its
                        // full 2..24 range, which is why only this mode broke.)
                        csum = csum ^ {6'b0, diag_lvl[17:16]};
                        csum = csum ^ diag_lvl[15:8];
                        csum = csum ^ diag_lvl[7:0];
                        pkt[25] <= csum;

                        pkt_idx <= 7'd0;
                        state   <= 2'd1;
                    end else begin
                        //---- build the raw waveform packet (A5 5A) ----
                        pkt[0] <= 8'hA5;
                        pkt[1] <= 8'h5A;
                        pkt[2] <= frame_cnt[7:0];
                        pkt[3] <= 8'h07;

                        pkt[4]  <= ch0[DATA_W-1 -: 8];
                        pkt[5]  <= ch0[DATA_W-9 -: 8];
                        pkt[6]  <= ch0[DATA_W-17 -: 8];

                        pkt[7]  <= ch1[DATA_W-1 -: 8];
                        pkt[8]  <= ch1[DATA_W-9 -: 8];
                        pkt[9]  <= ch1[DATA_W-17 -: 8];

                        pkt[10] <= ch2[DATA_W-1 -: 8];
                        pkt[11] <= ch2[DATA_W-9 -: 8];
                        pkt[12] <= ch2[DATA_W-17 -: 8];

                        pkt[13] <= ch3[DATA_W-1 -: 8];
                        pkt[14] <= ch3[DATA_W-9 -: 8];
                        pkt[15] <= ch3[DATA_W-17 -: 8];

                        pkt[16] <= ch4[DATA_W-1 -: 8];
                        pkt[17] <= ch4[DATA_W-9 -: 8];
                        pkt[18] <= ch4[DATA_W-17 -: 8];

                        pkt[19] <= ch5[DATA_W-1 -: 8];
                        pkt[20] <= ch5[DATA_W-9 -: 8];
                        pkt[21] <= ch5[DATA_W-17 -: 8];

                        pkt[22] <= ch6[DATA_W-1 -: 8];
                        pkt[23] <= ch6[DATA_W-9 -: 8];
                        pkt[24] <= ch6[DATA_W-17 -: 8];

                        pkt[25] <= 8'h00;   // checksum patched below

                        // checksum over bytes 2..24
                        csum = 8'h00;
                        csum = csum ^ frame_cnt[7:0];
                        csum = csum ^ 8'h07;
                        csum = csum ^ ch0[DATA_W-1 -: 8] ^ ch0[DATA_W-9 -: 8] ^ ch0[DATA_W-17 -: 8];
                        csum = csum ^ ch1[DATA_W-1 -: 8] ^ ch1[DATA_W-9 -: 8] ^ ch1[DATA_W-17 -: 8];
                        csum = csum ^ ch2[DATA_W-1 -: 8] ^ ch2[DATA_W-9 -: 8] ^ ch2[DATA_W-17 -: 8];
                        csum = csum ^ ch3[DATA_W-1 -: 8] ^ ch3[DATA_W-9 -: 8] ^ ch3[DATA_W-17 -: 8];
                        csum = csum ^ ch4[DATA_W-1 -: 8] ^ ch4[DATA_W-9 -: 8] ^ ch4[DATA_W-17 -: 8];
                        csum = csum ^ ch5[DATA_W-1 -: 8] ^ ch5[DATA_W-9 -: 8] ^ ch5[DATA_W-17 -: 8];
                        csum = csum ^ ch6[DATA_W-1 -: 8] ^ ch6[DATA_W-9 -: 8] ^ ch6[DATA_W-17 -: 8];
                        pkt[25] <= csum;

                        pkt_idx <= 7'd0;
                        state   <= 2'd1;
                    end
                end
            end
            //----------------------------------------------------------
            2'd1: begin
                // wait one cycle for the buffer to settle, then start sending
                state <= (PKT_MODE == 2) ? 2'd3 : 2'd2;
            end
            //----------------------------------------------------------
            2'd2: begin
                if (!tx_busy && !tx_start) begin
                    if (pkt_idx == PKT_LEN-1) begin
                        tx_start   <= 1'b1;
                        tx_data    <= pkt[pkt_idx];
                        bytes_sent <= bytes_sent + 1'b1;
                        pkt_idx    <= 6'd0;
                        state      <= 2'd0;
                    end else begin
                        tx_start   <= 1'b1;
                        tx_data    <= pkt[pkt_idx];
                        bytes_sent <= bytes_sent + 1'b1;
                        pkt_idx    <= pkt_idx + 1'b1;
                    end
                end
            end
            //----------------------------------------------------------
            2'd3: begin
                // ASCII diagnostic line: send_len characters out of txt[].
                // Same handshake as state 2 -- one byte per tx_busy cycle.
                if (!tx_busy && !tx_start) begin
                    //  txt[pkt_idx], NOT txt[pkt_idx[5:0]]: the index was
                    //  narrowed to six bits when every line fitted in 64
                    //  bytes, so a 77-character line wrapped back to
                    //  "NO DIRECTION" after 64 bytes.  On a terminal that
                    //  looks like a truncated line followed by a new one,
                    //  which is exactly the kind of thing that gets blamed
                    //  on the cable.  Found by tb_uart_nodir 2026-11-01.
                    if (pkt_idx == send_len - 1'b1) begin
                        tx_start   <= 1'b1;
                        tx_data    <= txt[pkt_idx];
                        bytes_sent <= bytes_sent + 1'b1;
                        pkt_idx    <= 7'd0;
                        state      <= 2'd0;
                    end else begin
                        tx_start   <= 1'b1;
                        tx_data    <= txt[pkt_idx];
                        bytes_sent <= bytes_sent + 1'b1;
                        pkt_idx    <= pkt_idx + 1'b1;
                    end
                end
            end
            endcase
        end
    end

endmodule
