//-----------------------------------------------------------------------------
// audio_uart_tx.v
//   Minimal UART transmitter, 8N1.  Sends one byte per 'tx_start'.
//   Used to stream the captured mic PCM to the PC over the board UART
//   (Tang MEGA 60K: 'txp' = U15, LVCMOS33 3.3V).
//
//   CLK_HZ and BAUD set the divisor.  Default 50 MHz / 115200 baud.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module audio_uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,     // pulse: load tx_data and begin
    input  wire [7:0] tx_data,
    output reg        tx,           // serial out (idle high)
    output reg        tx_busy       // high while shifting
);
    localparam integer DIV = CLK_HZ / BAUD;
    localparam integer W   = (DIV < 2) ? 1 : $clog2(DIV);

    reg [W-1:0]  baud_cnt;
    reg [3:0]    bit_idx;           // 0=start,1..8=data,9=stop
    reg [9:0]    sh;                // {stop,data[7:0],start}

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx       <= 1'b1;
            tx_busy  <= 1'b0;
            baud_cnt <= { W {1'b0} };
            bit_idx  <= 4'd0;
            sh       <= 10'h3FF;
        end else if (!tx_busy) begin
            tx <= 1'b1;
            if (tx_start) begin
                sh       <= { 1'b1, tx_data, 1'b0 };   // stop, data, start
                tx_busy  <= 1'b1;
                bit_idx  <= 4'd0;
                baud_cnt <= { W {1'b0} };
                tx       <= 1'b0;                      // start bit
            end
        end else begin
            if (baud_cnt == DIV-1) begin
                baud_cnt <= { W {1'b0} };
                if (bit_idx == 4'd9) begin
                    tx      <= 1'b1;
                    tx_busy <= 1'b0;
                end else begin
                    bit_idx <= bit_idx + 1'b1;
                    tx      <= sh[bit_idx + 1'b1];
                end
            end else begin
                baud_cnt <= baud_cnt + 1'b1;
            end
        end
    end
endmodule
