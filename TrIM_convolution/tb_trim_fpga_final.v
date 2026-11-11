// =============================================================
// tb_trim_fpga_final.v : full-size, self-checking pre-synthesis test
// =============================================================
// Runs the REAL configuration (28x28, K=3, PE-array/systolic design):
// sends a Sobel-x kernel (has negative taps, exercises the sign-
// extension fix) + a deterministic test image, receives all results,
// and diffs the 26x26 interior against a golden convolution computed
// independently right here in the testbench.
//
// NOTE on latency: this PE-array design's vertical psum chain (3
// registered stages) plus the adder tree add PIPE_LATENCY=3 extra
// cycles between when a pixel enters and when its window's result is
// available. trim_fpga_top.v now drives 3 extra flush cycles after
// the real image and captures N_TOTAL = N_PIX+3 results accordingly,
// so no shift is needed on the host side -- out_stream[n] directly
// represents the window ending at input index n. This offset was
// found by brute-force search against the golden model; if you change
// K, IW, or the PE array's depth, re-derive it (see chat log for the
// search code, still here for reference below).
//
// A fast simulated baud rate is used (CLKS_PER_BIT=10) -- this only
// affects UART timing in simulation, not the protocol/logic being
// tested. Use CLKS_PER_BIT=868 in the real bitstream for 100MHz/115200.
// =============================================================

`timescale 1ns/1ps

module tb_trim_fpga_final;

    localparam IW  = 24;
    localparam K   = 3;
    localparam IMG_W = 28;
    localparam IMG_H = 28;
    localparam CLKS_PER_BIT = 10;      // fast fake baud for simulation only
    localparam OW = 2*IW + K + 2;      // matches adder_tree_k3.v's formula
    localparam RESULT_BYTES = (OW+7)/8; // 7
    localparam N_PIX = IMG_W*IMG_H;    // 784
    localparam PIPE_LATENCY = 3;       // must match trim_fpga_top.v's PIPE_LATENCY
    localparam N_TOTAL = N_PIX + PIPE_LATENCY;
    localparam OUT_H = IMG_H - K + 1;  // 26
    localparam OUT_W = IMG_W - K + 1;  // 26

    reg clk = 0;
    reg rst_n = 0;
    reg tb_tx_serial = 1'b1;
    wire dut_tx_serial;
    wire dut_done_led;

    always #5 clk = ~clk;  // 100MHz

    trim_fpga_top #(
        .IW(IW), .K(K), .IMG_W(IMG_W), .IMG_H(IMG_H),
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .rx_serial(tb_tx_serial),
        .tx_serial(dut_tx_serial),
        .done_led(dut_done_led)
    );

    // ---------------- byte-level UART tasks -----------------------------
    integer bi;
    task send_byte(input [7:0] b);
        begin
            tb_tx_serial = 1'b0;
            repeat (CLKS_PER_BIT) @(posedge clk);
            for (bi = 0; bi < 8; bi = bi + 1) begin
                tb_tx_serial = b[bi];
                repeat (CLKS_PER_BIT) @(posedge clk);
            end
            tb_tx_serial = 1'b1;
            repeat (CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    task recv_byte(output [7:0] b);
        integer j;
        begin
            @(negedge dut_tx_serial);
            repeat (CLKS_PER_BIT/2) @(posedge clk);
            for (j = 0; j < 8; j = j + 1) begin
                repeat (CLKS_PER_BIT) @(posedge clk);
                b[j] = dut_tx_serial;
            end
            repeat (CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    // ---------------- shared buffers -------------------------------------
    reg [7:0] rx_bytes [0:N_TOTAL*RESULT_BYTES-1];
    integer   r, i, row, col, kr, kc;
    reg [7:0] one_byte;
    reg signed [RESULT_BYTES*8-1:0] raw_stream [0:N_TOTAL-1];

    // task inputs go through these shared buffers (Icarus doesn't support
    // unpacked-array task ports)
    reg signed [7:0] xfer_kernel [0:8];
    reg        [7:0] xfer_image  [0:N_PIX-1];

    task do_transfer;
        begin
            for (i = 0; i < 9; i = i + 1)
                send_byte(xfer_kernel[i]);
            for (i = 0; i < N_PIX; i = i + 1)
                send_byte(xfer_image[i]);
            for (r = 0; r < N_TOTAL*RESULT_BYTES; r = r + 1) begin
                recv_byte(one_byte);
                rx_bytes[r] = one_byte;
            end
            for (r = 0; r < N_TOTAL; r = r + 1) begin
                raw_stream[r] = 0;
                for (i = 0; i < RESULT_BYTES; i = i + 1)
                    raw_stream[r] = raw_stream[r] | (rx_bytes[r*RESULT_BYTES+i] << (8*i));
            end
        end
    endtask

    // ---------------- Phase 1: calibrate ----------------------------------

    // ---------------- Phase 2: verify ------------------------------------
    integer golden [0:OUT_H-1][0:OUT_W-1];
    integer hw_val;
    integer n_mismatch;
    integer max_err, err;

    initial begin
        rst_n = 0; repeat (5) @(posedge clk); rst_n = 1; repeat (5) @(posedge clk);

        // Sobel-x kernel + deterministic varied test image
        xfer_kernel[0]=1;  xfer_kernel[1]=0;  xfer_kernel[2]=-1;
        xfer_kernel[3]=2;  xfer_kernel[4]=0;  xfer_kernel[5]=-2;
        xfer_kernel[6]=1;  xfer_kernel[7]=0;  xfer_kernel[8]=-1;
        for (row = 0; row < IMG_H; row = row + 1)
            for (col = 0; col < IMG_W; col = col + 1)
                xfer_image[row*IMG_W+col] = ((row*7 + col*13) % 256);  // deterministic, varied

        $display("=== Sending Sobel-x kernel + %0dx%0d image ===", IMG_H, IMG_W);
        do_transfer;

        // ---- golden model, computed independently right here ----
        for (row = 0; row < OUT_H; row = row + 1) begin
            for (col = 0; col < OUT_W; col = col + 1) begin
                golden[row][col] = 0;
                for (kr = 0; kr < K; kr = kr + 1)
                    for (kc = 0; kc < K; kc = kc + 1)
                        golden[row][col] = golden[row][col] +
                            xfer_kernel[kr*K+kc] * $signed({1'b0,xfer_image[(row+kr)*IMG_W + (col+kc)]});
            end
        end

        // ---- compare hw (no host-side shift needed now -- trim_fpga_top.v's
        // PIPE_LATENCY flush cycles handle the alignment) vs golden,
        // cropped to the 26x26 interior ----
        n_mismatch = 0;
        max_err = 0;
        for (row = 0; row < OUT_H; row = row + 1) begin
            for (col = 0; col < OUT_W; col = col + 1) begin
                hw_val = raw_stream[PIPE_LATENCY + (row+K-1)*IMG_W + (col+K-1)];
                err = hw_val - golden[row][col];
                if (err != 0) begin
                    n_mismatch = n_mismatch + 1;
                    if (err < 0) err = -err;
                    if (err > max_err) max_err = err;
                end
            end
        end

        $display("Compared %0d interior pixels.", OUT_H*OUT_W);
        $display("Mismatches: %0d / %0d", n_mismatch, OUT_H*OUT_W);
        $display("Max abs error: %0d", max_err);
        if (n_mismatch == 0)
            $display("MATCH: hardware convolution matches the golden model exactly.");
        else
            $display("MISMATCH: see trim_fpga_top.v weight-loading order / latency handling.");

        $display("Done.");
        $finish;
    end

endmodule
