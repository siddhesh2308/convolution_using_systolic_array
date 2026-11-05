// Top_Stage2_MxN.v - Stage 2 integration for a ROWS x COLS systolic array.
//
// Generalizes Top_Stage2_4x4: the array no longer has to be square.
//   - Weight matrix:      ROWS x COLS   (loaded once, weight-stationary)
//   - Activation matrix:  ROWS x ROWS   (square - one row-buffer per PE row,
//                          each buffer holds ROWS entries, same as before)
//   - Output matrix:      ROWS x COLS
//
// Timing note: the skewed row-buffer read pattern is unchanged from the
// working 4x4 design (row r starts reading at cc>=r). The output write
// timing below is a *derived, general* formula, not a copy of the old
// hand-found 4x4 magic numbers - it was checked to reproduce the exact
// 4x4 numbers (cc = n+4, n+6, n+7, n+8 for compute_done=11) when
// ROWS=COLS=4, but you should still confirm it against a waveform/
// testbench for whatever ROWS/COLS you actually use, the same way the
// original 4x4 numbers were confirmed by simulation.
//
// Derivation, for reference:
//   - Row r's buffer address during the cycle where cc==v is
//     clamp(v-r, 0, ROWS-1)  (skewed start, then increments once/cycle).
//   - Because the Buffer's read is registered AND its address update is
//     registered, element 0 of each row sits on a_row_in for 2 cycles
//     before advancing; elements 1..ROWS-1 each get exactly 1 cycle.
//   - That "extra cycle" for element 0 is what produces the uneven gap
//     you see below (address 0 arrives, then a 2-cycle gap, then steady
//     1-cycle-apart delivery for the rest).
//   - Column c's PE chain only depends on c and ROWS, not on COLS, so the
//     per-column formula holds for any COLS.
//
//   write_cycle(c, a) = c + ROWS + (a == 0 ? 0 : a + 1),  for a = 0..ROWS-1
//   compute_done_cycle = write_cycle(COLS-1, ROWS-1) = (COLS-1) + 2*ROWS
`timescale 1ns/1ps
module Top_Stage2_MxN #(
    parameter WIDTH = 24,
    parameter ROWS  = 3,   // array rows / activation matrix side (square)
    parameter COLS  = 3    // array columns / output width
)(
    input  wire clk,
    input  wire rst,
    input  wire start,

    // single ROWS*COLS-entry weight memory, loaded externally before start
    input  wire ext_we_w,
    input  wire [$clog2(ROWS*COLS)-1:0] ext_addr_w,
    input  wire signed [WIDTH-1:0] ext_data_w,

    // ROWS row buffers, ROWS entries each - flattened, one slice per row
    input  wire [ROWS-1:0]                 ext_we_row_flat,
    input  wire [ROWS*$clog2(ROWS)-1:0]    ext_addr_row_flat,
    input  wire [ROWS*WIDTH-1:0]           ext_data_row_flat,

    // COLS output column buffers, ROWS entries each - same external
    // read address broadcast to all of them (matches the old behavior)
    input  wire [$clog2(ROWS)-1:0]         ext_addr_col,
    output wire [COLS*WIDTH-1:0]           ext_data_col_flat,

    output wire done
);

    localparam WADDR = $clog2(ROWS*COLS);
    localparam RADDR = $clog2(ROWS);

    wire load_en, compute_en, store_en;
    reg  load_done, compute_done, store_done;

    Controller ctrl (
        .clk(clk), .rst(rst), .start(start),
        .load_done(load_done), .compute_done(compute_done), .store_done(store_done),
        .load_en(load_en), .compute_en(compute_en), .store_en(store_en), .done(done)
    );

    // ---- Weight memory + serial loader ----
    wire signed [WIDTH-1:0] wbuf_data_out;
    reg [WADDR-1:0] load_addr;
    Buffer #(.WIDTH(WIDTH), .DEPTH(ROWS*COLS), .ADDR_WIDTH(WADDR)) wbuf (
        .clk(clk), .rst(rst), .we(ext_we_w), .addr(ext_we_w ? ext_addr_w : load_addr),
        .data_in(ext_data_w), .data_out(wbuf_data_out));

    reg [WADDR:0] load_cnt; // needs to count up to ROWS*COLS+1
    reg signed [WIDTH-1:0] w_stage [0:ROWS*COLS-1];
    integer li;
    reg load_weight;

    always @(posedge clk) begin
        if (rst || !load_en) begin
            load_addr   <= 0;
            load_cnt    <= 0;
            load_weight <= 0;
            load_done   <= 0;
        end else begin
            load_addr <= (load_addr < ROWS*COLS-1) ? load_addr + 1 : load_addr;
            if (load_cnt >= 1 && load_cnt <= ROWS*COLS)
                w_stage[load_cnt-1] <= wbuf_data_out;
            load_cnt    <= load_cnt + 1;
            load_weight <= (load_cnt == ROWS*COLS);   // pulse once all weights are staged
            load_done   <= (load_cnt == ROWS*COLS+1);
        end
    end

    // flatten staged weights (row-major: index = r*COLS+c) onto the array's bus
    wire signed [ROWS*COLS*WIDTH-1:0] weight_flat;
    genvar wi;
    generate
        for (wi = 0; wi < ROWS*COLS; wi = wi + 1) begin: WEIGHT_FLATTEN
            assign weight_flat[wi*WIDTH +: WIDTH] = w_stage[wi];
        end
    endgenerate

    // ---- Row buffers (ROWS of them, ROWS entries each) ----
    wire signed [WIDTH-1:0] row_data_out [0:ROWS-1];
    reg  [RADDR-1:0]        row_addr     [0:ROWS-1];
    wire signed [WIDTH-1:0] a_row_in     [0:ROWS-1];

    genvar rr;
    generate
        for (rr = 0; rr < ROWS; rr = rr + 1) begin: ROW_BUF
            Buffer #(.WIDTH(WIDTH), .DEPTH(ROWS), .ADDR_WIDTH(RADDR)) buf_inst (
                .clk(clk), .rst(rst),
                .we(ext_we_row_flat[rr]),
                .addr(ext_we_row_flat[rr] ? ext_addr_row_flat[rr*RADDR +: RADDR] : row_addr[rr]),
                .data_in(ext_data_row_flat[rr*WIDTH +: WIDTH]),
                .data_out(row_data_out[rr])
            );
            assign a_row_in[rr] = (cc >= rr) ? row_data_out[rr] : {WIDTH{1'b0}};
        end
    endgenerate

    reg [31:0] cc; // free-running compute-phase cycle counter
    integer ri;
    always @(posedge clk) begin
        if (rst || !compute_en) begin
            cc <= 0;
            for (ri = 0; ri < ROWS; ri = ri + 1) row_addr[ri] <= 0;
        end else begin
            cc <= cc + 1;
            for (ri = 0; ri < ROWS; ri = ri + 1)
                if (cc >= ri && row_addr[ri] < ROWS-1) row_addr[ri] <= row_addr[ri] + 1;
        end
    end

    wire signed [ROWS*WIDTH-1:0] a_in_flat;
    genvar ai;
    generate
        for (ai = 0; ai < ROWS; ai = ai + 1) begin: A_FLATTEN
            assign a_in_flat[ai*WIDTH +: WIDTH] = a_row_in[ai];
        end
    endgenerate

    wire signed [COLS*WIDTH-1:0] psum_out_flat;

    Systolic_Array #(.WIDTH(WIDTH), .ROWS(ROWS), .COLS(COLS)) sa (
        .clk(clk), .rst(rst), .load_weight(load_weight),
        .weight_flat(weight_flat), .a_in_flat(a_in_flat),
        .psum_out_flat(psum_out_flat)
    );

    // compute_done: see derivation in the header comment
    always @(posedge clk) begin
        if (rst) compute_done <= 0;
        else compute_done <= compute_en && (cc == (COLS-1) + 2*ROWS);
    end

    // ---- Output column buffers (COLS of them, ROWS entries each) ----
    wire signed [WIDTH-1:0] col_data_out [0:COLS-1];
    reg                     col_we       [0:COLS-1];
    reg  [RADDR-1:0]        col_wa       [0:COLS-1];

    integer ci;
    always @(*) begin
        for (ci = 0; ci < COLS; ci = ci + 1) begin
            // write_cycle(c,0) = c+ROWS ; write_cycle(c,a>=1) = c+ROWS+a+1
            col_we[ci] = compute_en &&
                         ( (cc == ci + ROWS) ||
                           (cc >= ci + ROWS + 2 && cc <= ci + 2*ROWS) );
            col_wa[ci] = (cc == ci + ROWS) ? {RADDR{1'b0}}
                                            : (cc - (ci + ROWS + 1));
        end
    end

    genvar cb;
    generate
        for (cb = 0; cb < COLS; cb = cb + 1) begin: COL_BUF
            Buffer #(.WIDTH(WIDTH), .DEPTH(ROWS), .ADDR_WIDTH(RADDR)) buf_inst (
                .clk(clk), .rst(rst),
                .we(col_we[cb]),
                .addr(col_we[cb] ? col_wa[cb] : ext_addr_col),
                .data_in(psum_out_flat[cb*WIDTH +: WIDTH]),
                .data_out(col_data_out[cb])
            );
            assign ext_data_col_flat[cb*WIDTH +: WIDTH] = col_data_out[cb];
        end
    endgenerate

    always @(posedge clk) begin
        if (rst) store_done <= 0;
        else store_done <= store_en;
    end

endmodule
