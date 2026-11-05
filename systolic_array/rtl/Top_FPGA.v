// Top_FPGA.v - board-level top for the Nexys A7.
// Loads weights+activations over UART, runs the ROWS x COLS accelerator,
// then automatically streams all ROWS*COLS output values back over a
// second UART line, followed by a 4-byte clock-cycle count measured from
// start to done.
//
// Generalized from the fixed 4x4 version: change ROWS/COLS below (and
// resynthesize) to retarget the array size - no other edits needed here.
`timescale 1ns/1ps
module Top_FPGA #(
    parameter WIDTH     = 24,
    parameter ROWS      = 3,
    parameter COLS      = 3,
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire clk,          // 100 MHz board clock
    input  wire rst,          // active-high reset (map to a board button)
    input  wire uart_rx_pin,  // host -> board (weights + activations)
    output wire uart_tx_pin,  // board -> host (results + timing)

    output wire done,         // pulses once per full compute; wire to an LED
    output wire sending       // high while results are streaming out; wire to an LED
);

    localparam WADDR = $clog2(ROWS*COLS);
    localparam RADDR = $clog2(ROWS);

    // ---- loader: host -> board ----
    wire                     we_w;
    wire [WADDR-1:0]         addr_w;
    wire signed [WIDTH-1:0]  data_w;

    wire [ROWS-1:0]              we_row_flat;
    wire [ROWS*RADDR-1:0]        addr_row_flat;
    wire signed [ROWS*WIDTH-1:0] data_row_flat;

    wire start_pulse, load_complete;

    UART_Loader #(.WIDTH(WIDTH), .ROWS(ROWS), .COLS(COLS), .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) loader (
        .clk(clk), .rst(rst), .rx(uart_rx_pin),
        .ext_we_w(we_w), .ext_addr_w(addr_w), .ext_data_w(data_w),
        .ext_we_row_flat(we_row_flat), .ext_addr_row_flat(addr_row_flat), .ext_data_row_flat(data_row_flat),
        .start(start_pulse), .load_complete(load_complete)
    );

    // ---- accelerator ----
    wire [RADDR-1:0]              col_addr;
    wire signed [COLS*WIDTH-1:0]  col_out_flat;

    Top_Stage2_MxN #(.WIDTH(WIDTH), .ROWS(ROWS), .COLS(COLS)) accel (
        .clk(clk), .rst(rst), .start(start_pulse),
        .ext_we_w(we_w), .ext_addr_w(addr_w), .ext_data_w(data_w),
        .ext_we_row_flat(we_row_flat), .ext_addr_row_flat(addr_row_flat), .ext_data_row_flat(data_row_flat),
        .ext_addr_col(col_addr), .ext_data_col_flat(col_out_flat),
        .done(done)
    );

    // ---- cycle counter: measures start_pulse -> done latency in clocks ----
    // Note: this times the FULL load+compute+store pipeline (i.e. how long
    // the accelerator takes end-to-end once weights/activations are already
    // sitting in its buffers), not just the systolic array's MAC phase.
    reg [31:0] cycle_count;
    reg [31:0] latched_cycle_count;
    reg        counting;

    always @(posedge clk) begin
        if (rst) begin
            cycle_count         <= 32'd0;
            latched_cycle_count <= 32'd0;
            counting            <= 1'b0;
        end else begin
            if (start_pulse) begin
                cycle_count <= 32'd0;
                counting    <= 1'b1;
            end else if (counting) begin
                cycle_count <= cycle_count + 32'd1;
                if (done) begin
                    latched_cycle_count <= cycle_count;
                    counting            <= 1'b0;
                end
            end
        end
    end

    // ---- result sender: board -> host ----
    Result_Sender #(.WIDTH(WIDTH), .ROWS(ROWS), .COLS(COLS), .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) sender (
        .clk(clk), .rst(rst), .start(done),
        .col_in_flat(col_out_flat),
        .addr_out(col_addr),
        .cycle_count_in(latched_cycle_count),
        .tx(uart_tx_pin), .busy(sending)
    );

endmodule
