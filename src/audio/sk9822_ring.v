//-----------------------------------------------------------------------------
// sk9822_ring.v
//   Drives the 12 RGB LEDs that sit on the Sipeed 6+1 microphone array board,
//   and uses them to show the solved direction of arrival.
//
//   HARDWARE  (verified against the Sipeed wiki, 2026-03)
//   ------------------------------------------------
//   The array board carries 12 x SK9822 LEDs arranged in a ring, CASCADED on a
//   SINGLE two-wire bus:
//
//       LED_CK  (FPC pin 9)   -> shift clock for all 12 LEDs
//       LED_DA  (FPC pin 10)  -> serial data, daisy-chained LED 1 -> LED 12
//
//   The wiki's wording "多个 LED 通过双信号线级联" means the bus is two wires
//   (clock + data); it does NOT mean there are two parallel data chains.  The
//   FPC pin table lists exactly one LED_CK and one LED_DA, and Sipeed's own
//   reference driver (sipeed_sk9822.c) pushes all twelve 32-bit words down a
//   single data line.  An earlier revision of this file wrongly assumed two
//   six-LED chains; that is corrected here.
//
//   On the Tang MEGA 60K these two signals are brought out as
//       sk9822_ck = W12      sk9822_da = AA9
//   and they sit in the SAME 1.5 V bank as the microphone lines, so they must
//   be constrained LVCMOS15 / BANK_VCCIO=1.5 like everything else on J1.
//
//   LED INDEXING AND SECTORS
//   ------------------------
//   The intended mapping is one LED per 30 degree sector:
//
//       sector k  ->  LED k,  k = 0..11,  LED k covers [ -180 + 30k, ... )
//
//   Which physical LED sits where is a board-layout question, not a protocol
//   one, so it is exposed as the LED_REVERSE parameter: set it to 1'b1 if the
//   ring sweeps the wrong way round on hardware.  It changes no timing.
//
//   PROTOCOL (SK9822; APA102-compatible superset)
//   ---------------------------------------------
//   A frame is:
//       32 zero bits          start frame
//       32 bits per LED       "111" + 5-bit global brightness + 8b B + 8b G + 8b R
//       32 one bits           end frame
//
//   Bits go out MSB first.  The data line is set up while the clock is LOW and
//   the clock is pulsed HIGH for the LED to sample, so transitions land on the
//   falling edge and the sample point is the rising edge.  The shift clock
//   runs at CLK_HZ -- slowed from 25 MHz to 2.5 MHz (2026-10-11) -- because
//   the bus is a 1.5 V bank driving 5 V-powered LED chips through an FPC
//   cable.  The SK9822 tolerates far more than 2.5 MHz, and margin is free.
//
//   BRIGHTNESS IS DELIBERATELY LOW.  Twelve LEDs at full white together draw
//   enough current to disturb a 1.5 V I/O bank that also carries the
//   microphone clock lines, which would appear as noise in exactly the signals
//   this project is trying to measure.  The default global brightness is 1/31
//   and typically only three LEDs are lit.
//
//   UPDATE POLICY
//   -------------
//   A frame is shifted only when 'new_dir' pulses (a fresh bearing has been
//   published) AND the minimum quiet period has elapsed.  Holding the LED
//   clock low between updates matters: shifting continuously would radiate a
//   25 MHz clock straight past the microphones.  One full frame is
//   (32 + 12*32 + 32) = 448 bits; the rest of the interval is silent.
//
//   WHAT THE RING SHOWS (2026-10-11, self-test flash added 2026-10-14)
//   ------------------------------------------------------------------
//     * power-up: ALL twelve LEDs light for BLANK_CYCLES -- the self-test
//       flash -- and then go dark.  Look for this FIRST on a new board:
//       "flash, then dark" proves the whole LED path works, while "a few
//       LEDs stuck on and nothing changes" means no frame is arriving at
//       all (the SK9822s are holding their power-on latches).  See the FSM
//       note below;
//     * a new_dir pulse (from top.v, while the audio energy is above
//       ANY_SND_E) shows that sector;
//     * no new_dir for BLANK_CYCLES -> dark, and the dark frame is re-sent
//       every BLANK_CYCLES after that, so nothing can stay stuck on.
//
//           ring lit  = sound, and these are the LEDs for its direction
//           ring dark = no sound
//
//   There is no idle pattern, no sweep and no second display mode.
//   Everything else is one job: shift out one frame per new direction, at
//   most one frame per HOLD_CYCLES, and keep the LED clock low the rest of
//   the time -- a continuously clocked 2.5 MHz line would radiate straight
//   into the microphones.
//
//   Global brightness is low on purpose: twelve LEDs at full white would
//   disturb the supply that also carries the microphone clocks.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module sk9822_ring #(
    parameter integer NLED        = 12,            // total LEDs on the ring
    parameter integer NSEC        = 12,            // bearing sectors to display
    parameter integer GLOBAL_BRI  = 1,             // 0..31, see the note above
    parameter integer CORE_HZ     = 50_000_000,
    parameter integer CLK_HZ      = 2_500_000,     // LED shift clock (see header)
    parameter integer HOLD_CYCLES = 1_000_000,     // min quiet gap (20 ms @ 50 MHz)
    parameter integer BLANK_CYCLES= 25_000_000,    // no new direction this long
                                                   // (0.5 s @ 50 MHz) -> go dark
    parameter         LED_REVERSE = 1'b0
) (
    input  wire        clk,
    input  wire        rst_n,

    // solved direction
    input  wire        new_dir,       // one pulse per new bearing
    input  wire [3:0]  sector,        // 0..NSEC-1
    input  wire [3:0]  active_span,   // LEDs either side of the centre to light

    output reg         led_clk,
    output reg         led_dat,

    // 1 while the ring is actually showing a direction (i.e. not blanked).
    // Reported on the UART "F" line so three LED numbers cannot be mistaken
    // for three LEDs that are on.
    output wire        lit
);

    //==================================================================
    //  Colour table: one distinctive colour per sector so the bearing can be
    //  read at a glance.  Packed {B,G,R} to match the shift order.
    //==================================================================
    function [23:0] sector_colour;
        input [3:0] s;
        begin
            case (s)
                4'd0 : sector_colour = 24'hFF_00_00; // red
                4'd1 : sector_colour = 24'hFF_40_00; // orange
                4'd2 : sector_colour = 24'hFF_A0_00; // amber
                4'd3 : sector_colour = 24'hFF_FF_00; // yellow
                4'd4 : sector_colour = 24'h80_FF_00; // yellow-green
                4'd5 : sector_colour = 24'h00_FF_00; // green
                4'd6 : sector_colour = 24'h00_FF_80; // spring
                4'd7 : sector_colour = 24'h00_FF_FF; // cyan
                4'd8 : sector_colour = 24'h00_80_FF; // azure
                4'd9 : sector_colour = 24'h00_00_FF; // blue
                4'd10: sector_colour = 24'h80_00_FF; // violet
                default: sector_colour = 24'hFF_00_FF; // magenta
            endcase
        end
    endfunction

    //==================================================================
    //  Ring distance between an LED index and the active sector, wrapped so
    //  that 0 and NSEC-1 are neighbours.
    //==================================================================
    function integer ring_dist;
        input integer a;
        input integer b;
        integer d;
        begin
            d = a - b;
            if (d < 0) d = -d;
            if (d > NSEC/2) d = NSEC - d;
            ring_dist = d;
        end
    endfunction

    //==================================================================
    //  Frame construction.
    //
    //  Transmission order on a daisy-chained SK9822 bus: the FIRST 32-bit
    //  word shifted out ends up in the LAST LED of the chain, and the word
    //  shifted out last lands in the LED nearest the driver.  Word slot w = 0
    //  is therefore transmitted first and is the far end of the ring.
    //==================================================================
    localparam integer DATA_BITS  = NLED * 32;
    localparam integer FRAME_BITS = 32 + DATA_BITS + 32;

    // Per-frame state, captured when a frame starts.
    reg [3:0] sec_hold;
    reg [3:0] span_hold;

    wire [23:0] col = sector_colour(sec_hold);

    // logical LED index (0..NLED-1) for a wire word slot
    function integer led_index;
        input integer w;                 // 0..NLED-1, transmission order
        begin
            led_index = LED_REVERSE ? (NLED-1-w) : w;
        end
    endfunction

    //------------------------------------------------------------------
    //  blank_run -- "the frame being shifted out is a blank frame".
    //
    //  It is the snapshot of blank_hold taken when a frame starts, so a
    //  direction arriving mid-frame cannot corrupt the frame in flight (see
    //  BLANKING in the header).
    //
    //  DECLARED HERE, AHEAD OF led_word(), ON PURPOSE (2026-10-15).  It used
    //  to be declared 60 lines further down, next to blank_hold, while
    //  led_word() reads it -- a forward reference.  Verilog-2001 requires a
    //  declaration before use and ModelSim rejects the whole file with
    //
    //      ** Error: sk9822_ring.v(187): (vlog-2730)
    //         Undefined variable: 'blank_run'.
    //
    //  Gowin's synthesiser accepted it, so the design always built and the
    //  defect was invisible: this file had never been simulated.  Moving the
    //  declaration cannot change behaviour -- declaration order has no
    //  semantic effect on a reg's value -- it only makes the file legal.
    //------------------------------------------------------------------
    reg        blank_run;

    // the 32-bit protocol word for wire slot w
    function [31:0] led_word;
        input integer w;
        integer idx;
        reg    on;
        begin
            idx = led_index(w);
            on  = (ring_dist(idx, sec_hold) <= span_hold);
            if (blank_run)
                led_word = {3'b111, GLOBAL_BRI[4:0], 24'h00_00_00};
            else if (on)
                led_word = {3'b111, GLOBAL_BRI[4:0], col};
            else
                led_word = {3'b111, GLOBAL_BRI[4:0], 24'h00_00_00};
        end
    endfunction

    //==================================================================
    //  Bit-rate tick: one tick per LED clock HALF period.
    //==================================================================
    localparam integer DIV = (CORE_HZ + CLK_HZ) / (2*CLK_HZ);

    reg [15:0] div_cnt;
    reg        half;          // toggles each tick: low phase, then high phase

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 16'd0;
            half    <= 1'b0;
        end else begin
            if (div_cnt == DIV[15:0] - 16'd1) begin
                div_cnt <= 16'd0;
                half    <= ~half;
            end else
                div_cnt <= div_cnt + 16'd1;
        end
    end

    //------------------------------------------------------------------
    //  PHASE-CHANGE PULSE  (fixed 2026-10-16)
    //
    //  'half' is a LEVEL that holds for DIV core cycles; it is NOT a
    //  single-cycle event.  The shift FSM used to branch on it directly:
    //
    //      if (half) begin led_clk <= 1'b1; bit_ix <= bit_ix + 1; end
    //
    //  so bit_ix advanced DIV times per LED clock half-period instead of
    //  once.  With DIV = 10 the frame was clocked out about ten times too
    //  fast and only roughly every tenth bit ever reached the LEDs, so the
    //  ring latched shifted garbage -- colour patterns that never matched
    //  sector_colour() and never responded to new_dir.
    //
    //  tb_sk9822 catches exactly this: with the old code the receiver model
    //  decoded no complete frame at all (no end frame ever appeared) and the
    //  latched words were values like 0xC50638, whose top bits are not the
    //  mandatory 3'b111 data-word header.
    //
    //  'tick' is the phase-change pulse and the ONLY moment at which the
    //  shift position may advance.
    //------------------------------------------------------------------
    reg  half_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) half_d <= 1'b0;
        else        half_d <= half;
    end
    wire tick = (half != half_d);

    //==================================================================
    //  Shift bookkeeping.
    //
    //  bit_ix counts BITS within the frame (not half-periods).  It advances
    //  once per full clock period, on the tick that raises the clock.
    //==================================================================
    localparam integer S_IDLE = 1'b0,
                       S_RUN  = 1'b1;

    reg        state;
    reg [9:0]  bit_ix;        // 0..FRAME_BITS-1
    reg [23:0] hold_cnt;

    // A new direction that arrived while the quiet gap was still running is
    // remembered here rather than dropped.  'new_dir' is a single-cycle pulse
    // from the solver, which refreshes far faster than the LED ring needs to,
    // so latching it both prevents a lost request and keeps the two rate
    // domains independent.  While pending, the LATEST request wins.
    reg        pending;
    reg [3:0]  pend_sec;
    reg [3:0]  pend_span;

    //------------------------------------------------------------------
    //  Blank-frame bookkeeping (see BLANKING in the header).
    //  blank_hold is the live "the display should be dark" state;
    //  blank_run is its snapshot taken when a frame starts -- declared
    //  further up, ahead of led_word(), which reads it.
    //------------------------------------------------------------------
    reg        blank_hold;
    localparam integer SCW = $clog2(BLANK_CYCLES + 1);
    localparam [SCW-1:0] BLANK_LAST = BLANK_CYCLES - 1;
    reg [SCW-1:0] stale_cnt;

    // Live "the ring is showing something" flag, for the UART report.
    // Without it a status line that prints three LED indices is ambiguous:
    // after RING_BLANK_MS of silence those three LEDs are DARK, and the
    // numbers would be describing a display that is not there.
    assign lit = ~blank_hold;

    // (The idle sweep used to live here: a single LED walking the ring while
    //  no bearing was published.  It was removed on request -- "there is no
    //  sound" is now expressed by a dark ring, and nothing else.)

    // Which 32-bit word is being sent, and which bit within it.  The first 32
    // bits are the start frame (all zero) and the last 32 the end frame (all
    // one), so both are handled without consulting the payload.
    wire        in_start = (bit_ix < 10'd32);
    wire        in_end   = (bit_ix >= 32 + DATA_BITS);

    wire [9:0]  data_ix  = bit_ix - 10'd32;              // 0..DATA_BITS-1
    wire [4:0]  word_sel = data_ix[9:5];                 // 0..NLED-1
    wire [4:0]  bit_sel  = 5'd31 - data_ix[4:0];         // MSB first

    wire [31:0] word_cur = led_word(word_sel[4:0]);

    // A single data line: zeros during the start frame, ones during the end
    // frame, payload bits in between.
    wire dat_next = in_start ? 1'b0 :
                    in_end   ? 1'b1 : word_cur[bit_sel];

    //==================================================================
    //  Main FSM.
    //
    //  One LED clock period is two half-ticks:
    //     half == 0 : clock LOW  -> present the data bit (it settles here)
    //     half == 1 : clock HIGH -> LED samples, and we advance one bit
    //  Updating data on the low phase and advancing on the high phase gives a
    //  clean setup/hold relationship regardless of the clock ratio.
    //==================================================================
    //------------------------------------------------------------------
    //  POWER-UP SELF-TEST FLASH
    //
    //  "A few LEDs stuck on and nothing changes" has two very different
    //  causes that look identical on the bench: the LED bus is broken (no
    //  frame ever arrives, so the SK9822s keep whatever they latched at
    //  power-on), or the bus is fine and every frame is simply dark.  So the
    //  first frame after reset lights ALL TWELVE LEDs, and the normal
    //  blanking takes it away BLANK_CYCLES later:
    //
    //      flash seen, then dark  -> the whole LED path works (clock, data,
    //                                format, cable).  Anything wrong after
    //                                that is upstream: mic data or the
    //                                ANY_SND_E energy gate.
    //      no flash, LEDs stuck   -> nothing reaches the ring.  Check the
    //                                LED_CK / LED_DAT pair (swapped? open?),
    //                                the common ground, and the array supply.
    //
    //  The idle state also RE-SENDS the dark frame every BLANK_CYCLES now,
    //  so a lost or corrupted frame can never leave LEDs lit for good.
    //------------------------------------------------------------------
    localparam [3:0] ALL_SPAN = 4'd7;   // ring_dist <= NSEC/2 = 6, so a span
                                        // of 7 covers every LED on the ring

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            bit_ix    <= 10'd0;
            hold_cnt  <= 24'd0;
            sec_hold  <= 4'd0;
            span_hold <= ALL_SPAN;
            pending   <= 1'b1;        // power-up frame: the self-test flash
            pend_sec  <= 4'd0;
            pend_span <= ALL_SPAN;    // all twelve LEDs
            blank_hold<= 1'b0;        // shown, not blanked
            blank_run <= 1'b0;
            stale_cnt <= {SCW{1'b0}};
            led_clk   <= 1'b0;
            led_dat   <= 1'b0;
        end else begin
            // quiet gap timer
            if (hold_cnt != HOLD_CYCLES[23:0])
                hold_cnt <= hold_cnt + 24'd1;

            // latch an incoming direction at any time; the newest one wins
            if (new_dir) begin
                pending    <= 1'b1;
                pend_sec   <= sector;
                pend_span  <= active_span;
                blank_hold <= 1'b0;
                stale_cnt  <= {SCW{1'b0}};
            end else if (!blank_hold) begin
                // No direction for BLANK_CYCLES -> dark from here on (this is
                // also what ends the power-up flash).
                if (stale_cnt == BLANK_LAST) begin
                    blank_hold <= 1'b1;
                    pending    <= 1'b1;
                    pend_sec   <= 4'd0;
                    pend_span  <= 4'd0;
                    stale_cnt  <= {SCW{1'b0}};
                end else
                    stale_cnt <= stale_cnt + 1'b1;
            end else begin
                // Idle: re-send the dark frame periodically instead of once,
                // so a lost/corrupted frame or an LED that latched garbage at
                // power-on is corrected within BLANK_CYCLES.
                if (stale_cnt == BLANK_LAST) begin
                    pending    <= 1'b1;
                    pend_sec   <= 4'd0;
                    pend_span  <= 4'd0;
                    stale_cnt  <= {SCW{1'b0}};
                end else
                    stale_cnt <= stale_cnt + 1'b1;
            end

            case (state)
            //----------------------------------------------------------
            S_IDLE: begin
                led_clk <= 1'b0;
                led_dat <= 1'b0;
                bit_ix  <= 10'd0;

                //  !half makes the frame start on a LOW phase.  Entering
                //  mid-phase emitted one extra clock edge, which shifted the
                //  entire payload by a bit -- and with a 32-bit start frame
                //  that shift lands inside the data words.
                if (pending && (hold_cnt >= HOLD_CYCLES[23:0]) && !half) begin
                    sec_hold  <= pend_sec;
                    span_hold <= pend_span;
                    blank_run <= blank_hold;   // dark iff no direction is live
                    pending   <= 1'b0;
                    hold_cnt  <= 24'd0;
                    state     <= S_RUN;
                end
            end

            //----------------------------------------------------------
            S_RUN: begin
                // clock follows the phase, so each phase lasts exactly DIV
                // core cycles and one LED clock period carries ONE bit
                led_clk <= half;

                // present the bit during the low phase; the LED samples it on
                // the rising edge.  Re-writing it every cycle of the low
                // phase is harmless (dat_next does not change until bit_ix
                // does) and removes any dependence on the phase boundary.
                if (!half)
                    led_dat <= dat_next;

                // advance on the rising edge only -- one bit per clock period
                if (tick && half) begin
                    if (bit_ix == FRAME_BITS[9:0] - 10'd1) begin
                        state  <= S_IDLE;
                        bit_ix <= 10'd0;
                    end else
                        bit_ix <= bit_ix + 10'd1;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
