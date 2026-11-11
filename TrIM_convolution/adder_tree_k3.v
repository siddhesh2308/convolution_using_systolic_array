// =============================================================
// adder_tree_k3.v : K=3 adder tree, ceil(log2(3)) = 2 stages
// =============================================================
// Sums the 3 column psums coming out of the top row (Row0) of the
// PE array into the slice's final convolution output, per Sec III-A:
// output width = 2*IW + K + ceil(log2(K)) bits.
// =============================================================

module adder_tree_k3 #(
    parameter IW   = 24,
    parameter PSW  = 2*IW + 3,          // per-column psum width (K=3)
    parameter OW   = PSW + 2            // ceil(log2(3)) = 2 extra bits
)(
    input                        clk,
    input                        rst_n,
    input                        in_valid,
    input  signed [PSW-1:0]      p0, p1, p2,   // column 0,1,2 psums
    output reg                   out_valid,
    output reg signed [OW-1:0]   conv_out
);

    wire signed [PSW:0]  stage1_sum  = p0 + p1;                 // +1 bit
    wire signed [OW-1:0] stage2_sum  = {{1{stage1_sum[PSW]}}, stage1_sum} +
                                        {{2{p2[PSW-1]}}, p2};    // +1 more bit

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            conv_out  <= {OW{1'b0}};
            out_valid <= 1'b0;
        end else begin
            conv_out  <= stage2_sum;
            out_valid <= in_valid;
        end
    end

endmodule
