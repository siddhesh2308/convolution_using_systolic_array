// =============================================================
// trim_slice_top.v : 3x3 systolic PE array (K=3)
// =============================================================

module trim_slice_top #(
    parameter IW  = 24,
    parameter K   = 3,
    parameter W   = 28,
    parameter PSW = 2*IW + K,
    parameter OW  = PSW + 2
)(
    input                    clk,
    input                    rst_n,

    input                    w_load,
    input      [K*IW-1:0]    w_row_in,     // {w2,w1,w0} for the row being loaded

    input                    in_valid,
    input      [IW-1:0]      i_ext,

    output                   out_valid,
    output signed [OW-1:0]   conv_out
);

    // ---------------- line buffers (row delay lines) -----------------------
    reg [IW-1:0] line_a [0:W-1];   // delays by W cycles  -> "row above"
    reg [IW-1:0] line_b [0:W-1];   // delays by 2W cycles -> "row 2 above"
    integer k;

    wire [IW-1:0] row_top_w = line_b[W-1];
    wire [IW-1:0] row_mid_w = line_a[W-1];
    wire [IW-1:0] row_bot_w = i_ext;

    // ---------------- 3-tap horizontal window per row (registered) ---------
    reg [IW-1:0] win_top [0:2];
    reg [IW-1:0] win_mid [0:2];
    reg [IW-1:0] win_bot [0:2];

    // "live" (this-cycle) window values -- slot0=newest, slot2=oldest
    wire [IW-1:0] new_top0 = row_top_w, new_top1 = win_top[0], new_top2 = win_top[1];
    wire [IW-1:0] new_mid0 = row_mid_w, new_mid1 = win_mid[0], new_mid2 = win_mid[1];
    wire [IW-1:0] new_bot0 = row_bot_w, new_bot1 = win_bot[0], new_bot2 = win_bot[1];

    // wire array wrapper so the bottom-row live taps can be indexed the
    // same way as mid_delay1[]/top_delay2[] in the generate block below
    wire [IW-1:0] bot_tap [0:2];
    assign bot_tap[0] = new_bot0;
    assign bot_tap[1] = new_bot1;
    assign bot_tap[2] = new_bot2;

    // ---------------- extra skew registers for the psum-chain timing -------
    // Row1 needs the live mid-tap value from 1 cycle ago; Row0 needs the
    // live top-tap value from 2 cycles ago. See header for the derivation.
    reg [IW-1:0] mid_delay1 [0:2];
    reg [IW-1:0] top_delay1 [0:2];
    reg [IW-1:0] top_delay2 [0:2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < W; k = k + 1) begin
                line_a[k] <= {IW{1'b0}};
                line_b[k] <= {IW{1'b0}};
            end
            win_top[0]<=0; win_top[1]<=0; win_top[2]<=0;
            win_mid[0]<=0; win_mid[1]<=0; win_mid[2]<=0;
            win_bot[0]<=0; win_bot[1]<=0; win_bot[2]<=0;
            mid_delay1[0]<=0; mid_delay1[1]<=0; mid_delay1[2]<=0;
            top_delay1[0]<=0; top_delay1[1]<=0; top_delay1[2]<=0;
            top_delay2[0]<=0; top_delay2[1]<=0; top_delay2[2]<=0;
        end else if (in_valid) begin
            for (k = W-1; k >= 1; k = k - 1) begin
                line_b[k] <= line_b[k-1];
                line_a[k] <= line_a[k-1];
            end
            line_b[0] <= line_a[W-1];
            line_a[0] <= i_ext;

            win_top[0] <= new_top0; win_top[1] <= new_top1; win_top[2] <= new_top2;
            win_mid[0] <= new_mid0; win_mid[1] <= new_mid1; win_mid[2] <= new_mid2;
            win_bot[0] <= new_bot0; win_bot[1] <= new_bot1; win_bot[2] <= new_bot2;

            // capture LIVE values into the skew chains (delayed use next cycle(s))
            mid_delay1[0] <= new_mid0; mid_delay1[1] <= new_mid1; mid_delay1[2] <= new_mid2;
            top_delay1[0] <= new_top0; top_delay1[1] <= new_top1; top_delay1[2] <= new_top2;
            top_delay2[0] <= top_delay1[0]; top_delay2[1] <= top_delay1[1]; top_delay2[2] <= top_delay1[2];
        end
    end

    // ---------------- weight ripple: Row0 (new data) -> Row1 -> Row2 -------
    wire [IW-1:0] w0_out [0:K-1];
    wire [IW-1:0] w1_out [0:K-1];

    // ---------------- PE array + psum chain (Row2 first -> Row0 -> adder) --
    wire signed [PSW-1:0] r2_psum [0:K-1];
    wire signed [PSW-1:0] r1_psum [0:K-1];
    wire signed [PSW-1:0] r0_psum [0:K-1];

    genvar c;
    generate
        for (c = 0; c < K; c = c + 1) begin : COLS
            // kernel column c maps to tap slot (K-1-c): c=0(left)->slot2(oldest)
            // c=K-1(right)->slot0(newest) -- matches raster streaming order.

            pe #(.IW(IW), .PSW(PSW)) u_pe_row0 (
                .clk(clk), .rst_n(rst_n),
                .w_load(w_load),
                .w_in (w_row_in[(c+1)*IW-1 -: IW]),   // fresh data from FSM each load cycle
                .w_out(w0_out[c]),                     // ripples to Row1 next cycle
                .in_valid(in_valid), .in_sel(2'b00),
                .i_ext(top_delay2[K-1-c]), .i_r({IW{1'b0}}), .i_d({IW{1'b0}}),
                .i_left(), 
                .psum_in(r1_psum[c]), .psum_out(r0_psum[c])
            );

            pe #(.IW(IW), .PSW(PSW)) u_pe_row1 (
                .clk(clk), .rst_n(rst_n),
                .w_load(w_load),
                .w_in (w0_out[c]),
                .w_out(w1_out[c]),                     // ripples to Row2 next cycle
                .in_valid(in_valid), .in_sel(2'b00),
                .i_ext(mid_delay1[K-1-c]), .i_r({IW{1'b0}}), .i_d({IW{1'b0}}),
                .i_left(),
                .psum_in(r2_psum[c]), .psum_out(r1_psum[c])
            );

            pe #(.IW(IW), .PSW(PSW)) u_pe_row2 (
                .clk(clk), .rst_n(rst_n),
                .w_load(w_load),
                .w_in (w1_out[c]),
                .w_out(),                              // last stage, unused
                .in_valid(in_valid), .in_sel(2'b00),
                .i_ext(bot_tap[K-1-c]), .i_r({IW{1'b0}}), .i_d({IW{1'b0}}),
                .i_left(),
                .psum_in({PSW{1'b0}}), .psum_out(r2_psum[c])   // starts the chain
            );
        end
    endgenerate

    // ---------------- final accumulation across the 3 columns ------------
    adder_tree_k3 #(.IW(IW), .PSW(PSW), .OW(OW)) u_adder_tree (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .p0(r0_psum[0]), .p1(r0_psum[1]), .p2(r0_psum[2]),
        .out_valid(out_valid), .conv_out(conv_out)
    );

endmodule
