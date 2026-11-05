// Buffer.v - synchronous register-array buffer 
`timescale 1ns/1ps
module Buffer #(
    parameter WIDTH      = 24,
    parameter DEPTH      = 16,
    parameter ADDR_WIDTH = 4   // must satisfy 2**ADDR_WIDTH >= DEPTH
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     we,       // write enable
    input  wire [ADDR_WIDTH-1:0]    addr,
    input  wire signed [WIDTH-1:0]  data_in,
    output reg  signed [WIDTH-1:0]  data_out
);

    reg signed [WIDTH-1:0] mem [0:DEPTH-1];
    integer i;

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < DEPTH; i = i + 1)
                mem[i] <= 0;
            data_out <= 0;
        end else begin
            if (we)
                mem[addr] <= data_in;
            data_out <= mem[addr];
        end
    end

endmodule
