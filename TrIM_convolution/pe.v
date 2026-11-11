// =============================================================
// pe.v : Processing Element (PE) for the TrIM Slice
// =============================================================
// One PE = 1 weight register + 1 MAC + input-source mux + psum pass-through.
//
// Data widths follow the TrIM paper's convention (Sec III-A):
//   input activations : IW bits, unsigned
//   weights           : IW bits, signed   (paper uses B for both; we keep
//                        one width parameter IW for simplicity/your 24-bit spec)
//   psum out          : PSW = 2*IW + K bits (signed), sized for the worst
//                        case accumulation down a column of K PEs.
//
// Weight loading:
//   When w_load=1, the PE captures w_in as its stationary weight AND
//   forwards its *previous* weight value on w_out (so weights ripple
//   top-to-bottom through a column over K cycles, per the paper).
//
// Input source select (in_sel):
//   00 : I_ext  -> input grabbed directly from the periphery (bottom row)
//   01 : I_R    -> input arriving from the PE to the right, same row
//   10 : I_D    -> input arriving diagonally from the RSRB below
//
// Psum path:
//   Each PE adds its local product to psum_in (coming from the PE in the
//   same column, one row below -- accumulation flows bottom row -> top
//   row per the paper's Fig. 3) and drives psum_out.
// =============================================================

module pe #(
    parameter IW  = 24,                 // input/weight bit width
    parameter PSW = 2*IW + 3            // psum width (K hard-coded to 3 here;
                                         // recompute if you change K)
)(
    input                   clk,
    input                   rst_n,

    // weight loading
    input                   w_load,
    input  signed [IW-1:0]  w_in,
    output signed [IW-1:0]  w_out,       // ripple to the PE below (same column)

    // input activations
    input                   in_valid,
    input        [1:0]      in_sel,      // 00=I_ext 01=I_R 10=I_D
    input        [IW-1:0]   i_ext,
    input        [IW-1:0]   i_r,
    input        [IW-1:0]   i_d,
    output reg   [IW-1:0]   i_left,      // registered output -> PE to the left (I_L)

    // partial sum chain (vertical, bottom row -> top row)
    input  signed [PSW-1:0] psum_in,
    output reg signed [PSW-1:0] psum_out
);

    reg signed [IW-1:0] weight_r;
    reg        [IW-1:0] sel_in_r;

    assign w_out = weight_r;

    // -------- weight register (stationary, ripples down while loading) ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            weight_r <= {IW{1'b0}};
        else if (w_load)
            weight_r <= w_in;
    end

    // -------- input mux + registered forward to the left neighbour --------
    always @(*) begin
        case (in_sel)
            2'b00:   sel_in_r = i_ext;
            2'b01:   sel_in_r = i_r;
            2'b10:   sel_in_r = i_d;
            default: sel_in_r = i_ext;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            i_left <= {IW{1'b0}};
        else if (in_valid)
            i_left <= sel_in_r;
    end

    // -------- MAC: psum_out = psum_in + (input * weight) ------------------
    wire signed [2*IW-1:0] product = $signed({1'b0, sel_in_r}) * weight_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            psum_out <= {PSW{1'b0}};
        else if (in_valid)
            psum_out <= psum_in + {{(PSW-2*IW){product[2*IW-1]}}, product};
    end

endmodule
