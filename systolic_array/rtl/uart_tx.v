// uart_tx.v - simple 8N1 UART transmitter.
// Pulse tx_start for 1 cycle with data_in stable; busy stays high for the
// whole frame (start bit + 8 data bits + stop bit). Don't pulse tx_start
// again until busy has gone low.
`timescale 1ns/1ps
module UART_TX #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       tx_start,
    input  wire [7:0] data_in,
    output reg        tx,
    output reg        busy
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    localparam IDLE  = 2'd0,
               START = 2'd1,
               DATA  = 2'd2,
               STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  data_reg;

    always @(posedge clk) begin
        if (rst) begin
            state   <= IDLE;
            tx      <= 1'b1; // idle line is high
            busy    <= 1'b0;
            clk_cnt <= 0;
            bit_idx <= 0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1'b1;
                    if (tx_start) begin
                        data_reg <= data_in;
                        busy     <= 1'b1;
                        clk_cnt  <= 0;
                        state    <= START;
                    end
                end

                START: begin
                    tx <= 1'b0; // start bit
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1;
                    end else begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= DATA;
                    end
                end

                DATA: begin
                    tx <= data_reg[bit_idx]; // LSB first
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1;
                    end else begin
                        clk_cnt <= 0;
                        if (bit_idx < 7) begin
                            bit_idx <= bit_idx + 1;
                        end else begin
                            bit_idx <= 0;
                            state   <= STOP;
                        end
                    end
                end

                STOP: begin
                    tx <= 1'b1; // stop bit
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1;
                    end else begin
                        clk_cnt <= 0;
                        busy    <= 1'b0;
                        state   <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
