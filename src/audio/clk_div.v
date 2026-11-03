//-----------------------------------------------------------------------------
// clk_div.v
//   Simple integer clock divider with 50% duty cycle.
//   Derives the I2S bit clock.  top.v uses it as the I2S MASTER clock: the
//   MSM261S4030H0 microphones on the Sipeed array board are SLAVES (their
//   SCK/WS come straight from J32 pins 8/7 and the board has no clock of its
//   own), so BCK has to be generated here.  DIV=16 -> 50MHz/16 = 3.125MHz,
//   and WS = BCK/64 = 48.83kHz sample rate.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module clk_div #(
    parameter integer DIV = 8           // output = input / DIV  (DIV>=2, even)
) (
    input  wire clk_in,
    input  wire rst_n,
    output reg  clk_out
);
    localparam integer W = (DIV < 2) ? 1 : $clog2(DIV);

    reg [W-1:0] cnt;

    always @(posedge clk_in or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= { W {1'b0} };
            clk_out <= 1'b0;
        end else if (cnt == (DIV/2 - 1)) begin
            cnt     <= { W {1'b0} };
            clk_out <= ~clk_out;
        end else begin
            cnt <= cnt + 1'b1;
        end
    end
endmodule
