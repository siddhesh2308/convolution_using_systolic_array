// uart_rx.v - simple 8N1 UART receiver
// Samples at the middle of each bit period based on CLKS_PER_BIT.
`timescale 1ns/1ps
module UART_RX #(
    parameter CLK_FREQ  = 100_000_000,  // Nexys A7 default 100 MHz
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst,        // synchronous active-high reset
    input  wire       rx,         // serial line (from FTDI/USB-UART)
    output reg  [7:0] data_out,   // received byte
    output reg        data_valid  // 1-cycle pulse when data_out is valid
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    localparam IDLE  = 2'd0,
               START = 2'd1,
               DATA  = 2'd2,
               STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shift_reg;

    // 2-flop synchronizer for the async rx pin
    reg rx_meta, rx_sync;
    always @(posedge clk) begin
        rx_meta <= rx;
        rx_sync <= rx_meta;
    end

    always @(posedge clk) begin
        if (rst) begin
            state      <= IDLE;
            clk_cnt    <= 0;
            bit_idx    <= 0;
            shift_reg  <= 0;
            data_out   <= 0;
            data_valid <= 0;
        end else begin
            data_valid <= 1'b0; // default; pulses high only in STOP

            case (state)
                IDLE: begin
                    clk_cnt <= 0;
                    bit_idx <= 0;
                    if (rx_sync == 1'b0) // start bit falling edge seen
                        state <= START;
                end

                START: begin
                    // wait half a bit period, then confirm still low (real start bit)
                    if (clk_cnt == (CLKS_PER_BIT/2)) begin
                        if (rx_sync == 1'b0) begin
                            clk_cnt <= 0;
                            state   <= DATA;
                        end else begin
                            state <= IDLE; // glitch, abort
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                DATA: begin
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1;
                    end else begin
                        clk_cnt          <= 0;
                        shift_reg[bit_idx] <= rx_sync; // LSB first
                        if (bit_idx < 7) begin
                            bit_idx <= bit_idx + 1;
                        end else begin
                            bit_idx <= 0;
                            state   <= STOP;
                        end
                    end
                end

                STOP: begin
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1;
                    end else begin
                        clk_cnt    <= 0;
                        data_out   <= shift_reg;
                        data_valid <= 1'b1; // one-cycle pulse
                        state      <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
