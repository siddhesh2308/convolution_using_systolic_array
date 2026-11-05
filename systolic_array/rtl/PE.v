// PE.v - Processing Element for the systolic array
// Weight-stationary MAC
// Data width: 24-bit signed 

`timescale 1ns/1ps
module PE #( parameter WIDTH = 24)
(
    input  wire                     clk,
    input  wire                     rst,          // synchronous active-high reset
    input  wire                     load_weight,  // latch B_in as the stationary weight
    input  wire signed [WIDTH-1:0]  A_in,
    input  wire signed [WIDTH-1:0]  B_in,         // weight value while load_weight=1
    input  wire signed [WIDTH-1:0]  psum_in,
    output reg  signed [WIDTH-1:0]  A_out,
    output reg  signed [WIDTH-1:0]  B_out,
    output reg  signed [WIDTH-1:0]  psum_out
);

    reg signed [WIDTH-1:0] weight_reg;
    wire signed [WIDTH-1:0] product;

    assign product = (|A_in && |weight_reg)? A_in * weight_reg: {WIDTH{1'b0}};
    always @(posedge clk) begin
        if (rst) begin
            weight_reg <= 0;
            A_out      <= 0;
            B_out      <= 0;
            psum_out   <= 0;
        end else begin
            if (load_weight)
                weight_reg <= B_in;
            
            A_out    <= A_in;
            B_out    <= B_in;
            psum_out <= psum_in + product;
        end
    end

endmodule
