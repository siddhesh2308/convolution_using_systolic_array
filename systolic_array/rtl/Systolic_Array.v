// Systolic_Array.v - weight-stationary systolic array, ROWS x COLS PEs.
//
// Generalized from the old fixed 4x4 array: ROWS and COLS are now
// independent parameters, so the weight matrix is ROWS x COLS and the
// output is (activation_rows) x COLS. All ports are flattened packed
// buses (index r*COLS+c for weights, index r for activation inputs,
// index c for psum outputs) so this works for ANY ROWS/COLS, not just
// a hardcoded set of named signals like w00..w33.
`timescale 1ns/1ps
module Systolic_Array #(
    parameter WIDTH = 24,
    parameter ROWS  = 3,   // number of PE rows    (= weight matrix rows / activation length)
    parameter COLS  = 3    // number of PE columns (= weight matrix cols / output width)
)(
    input  wire clk,
    input  wire rst,
    input  wire load_weight,

    // weight[r][c] lives at bits [(r*COLS+c)*WIDTH +: WIDTH], row-major
    input  wire signed [ROWS*COLS*WIDTH-1:0] weight_flat,

    // a_in[r] lives at bits [r*WIDTH +: WIDTH] - one activation stream per PE row
    input  wire signed [ROWS*WIDTH-1:0]      a_in_flat,

    // psum_out[c] lives at bits [c*WIDTH +: WIDTH] - one accumulated output per PE column
    output wire signed [COLS*WIDTH-1:0]      psum_out_flat
);

    wire signed [WIDTH-1:0] weight    [0:ROWS-1][0:COLS-1];
    wire signed [WIDTH-1:0] a_wire    [0:ROWS-1][0:COLS];
    wire signed [WIDTH-1:0] psum_wire [0:ROWS][0:COLS-1];

    genvar r, c;

    generate
        // unpack weights
        for (r = 0; r < ROWS; r = r + 1) begin: WEIGHT_UNPACK_ROW
            for (c = 0; c < COLS; c = c + 1) begin: WEIGHT_UNPACK_COL
                assign weight[r][c] = weight_flat[(r*COLS + c)*WIDTH +: WIDTH];
            end
        end

        // unpack activation inputs - one per PE row, entering at column 0
        for (r = 0; r < ROWS; r = r + 1) begin: A_UNPACK
            assign a_wire[r][0] = a_in_flat[r*WIDTH +: WIDTH];
        end

        // top boundary: psum entering row 0 is always zero
        for (c = 0; c < COLS; c = c + 1) begin: TOP_BOUNDARY
            assign psum_wire[0][c] = {WIDTH{1'b0}};
        end

        // the PE grid itself
        for (r = 0; r < ROWS; r = r + 1) begin: PE_ROW
            for (c = 0; c < COLS; c = c + 1) begin: PE_COL
                PE #(.WIDTH(WIDTH)) pe_inst (
                    .clk(clk), .rst(rst), .load_weight(load_weight),
                    .A_in(a_wire[r][c]), .B_in(weight[r][c]), .psum_in(psum_wire[r][c]),
                    .A_out(a_wire[r][c+1]), .B_out(), .psum_out(psum_wire[r+1][c])
                );
            end
        end

        // pack outputs - column c's fully-accumulated psum comes out of the last row
        for (c = 0; c < COLS; c = c + 1) begin: PSUM_PACK
            assign psum_out_flat[c*WIDTH +: WIDTH] = psum_wire[ROWS][c];
        end
    endgenerate

endmodule
