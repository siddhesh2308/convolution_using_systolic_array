// =============================================================
// trim_fpga_top.v : end-to-end FPGA wrapper for the TrIM slice
// =============================================================
// Protocol (host -> board -> host), all over one UART
// =============================================================

module trim_fpga_top #(
    parameter IW            = 24,
    parameter K             = 3,
    parameter IMG_W         = 28,
    parameter IMG_H         = 28,
    parameter CLKS_PER_BIT  = 868          // 100MHz / 115200 baud
)(
    input      clk,
    input      rst_n,
    input      rx_serial,
    output     tx_serial,
    output reg done_led    // goes high once results have been fully sent
);

    localparam PSW           = 2*IW + K;
    localparam OW            = PSW + 2;    // must match adder_tree_k3.v's OW formula
    localparam RESULT_BYTES = (OW + 7) / 8;
    localparam N_PIX        = IMG_W * IMG_H;
    // PIPE_LATENCY: this PE-array design's psum chain + adder tree need a
    // few extra "flush" cycles after the last real pixel for the final
    // few windows to fully propagate to conv_out (empirically confirmed
    // via brute-force search against a golden model -- see tb_trim_fpga_final.v).
    // Without this, the last few output rows are truncated.
    localparam PIPE_LATENCY  = 3;
    localparam N_TOTAL       = N_PIX + PIPE_LATENCY;   // total driven/captured samples
    localparam AW            = $clog2(N_TOTAL);   // address width for both BRAMs

    // ---------------- UART instances --------------------------------
    wire       rx_valid;
    wire [7:0] rx_byte;
    reg        tx_start;
    reg  [7:0] tx_byte;
    wire       tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx_serial(rx_serial),
        .rx_valid(rx_valid), .rx_byte(rx_byte)
    );

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_start(tx_start), .tx_byte(tx_byte),
        .tx_serial(tx_serial), .tx_busy(tx_busy)
    );

    // ---------------- storage ----------------------------------------
    reg signed [7:0]  kernel_mem [0:8];        // 9 signed kernel taps
    reg        [7:0]  img_mem    [0:N_PIX-1];  // unsigned pixels

    reg signed [RESULT_BYTES*8-1:0] out_mem [0:N_TOTAL-1];

    // ---------------- TrIM slice instance -----------------------------
    reg                w_load;
    reg  [K*IW-1:0]    w_row_in;
    reg                in_valid;
    reg  [IW-1:0]      i_ext;
    wire               out_valid;
    wire signed [OW-1:0] conv_out;

    trim_slice_top #(.IW(IW), .K(K), .W(IMG_W)) u_slice (
        .clk(clk), .rst_n(rst_n),
        .w_load(w_load), .w_row_in(w_row_in),
        .in_valid(in_valid), .i_ext(i_ext),
        .out_valid(out_valid), .conv_out(conv_out)
    );

    // ---------------- control FSM --------------------------------------
    localparam S_RECV_KERNEL = 0,
               S_RECV_IMAGE  = 1,
               S_LOAD_W_0    = 2,   // feed kernel row2
               S_LOAD_W_1    = 3,   // feed kernel row1
               S_LOAD_W_2    = 4,   // feed kernel row0
               S_COMPUTE     = 5,
               S_SEND_RESULT = 6;

    reg [2:0]  state;
    reg [3:0]  kbyte_cnt;                 // 0..8
    reg [AW:0] pix_cnt;                   // 0..N_PIX (recv + compute reuse)
    reg [AW:0] out_wr_ptr;                // results captured so far
    reg [AW:0] send_addr;                 // which result is being sent
    reg [2:0]  send_byte_idx;             // which byte of the current result
    reg        txn_done_pulse;            // 1-cycle pulse: last result byte just sent

    // ---------------- done_led: independent pulse-stretcher ---------------
 
    localparam DWELL_CYCLES = 5_000_000;  // ~50ms at 100MHz -- visible LED blink
    reg [22:0] led_timer;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_led  <= 1'b0;
            led_timer <= 0;
        end else if (txn_done_pulse) begin
            done_led  <= 1'b1;
            led_timer <= DWELL_CYCLES - 1;
        end else if (led_timer != 0) begin
            led_timer <= led_timer - 1'b1;
        end else begin
            done_led <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_RECV_KERNEL;
            kbyte_cnt     <= 0;
            pix_cnt       <= 0;
            out_wr_ptr    <= 0;
            send_addr     <= 0;
            send_byte_idx <= 0;
            w_load        <= 1'b0;
            in_valid      <= 1'b0;
            tx_start      <= 1'b0;
            txn_done_pulse<= 1'b0;
        end else begin
            tx_start       <= 1'b0;   // default: single-cycle pulse
            w_load         <= 1'b0;
            in_valid       <= 1'b0;
            txn_done_pulse <= 1'b0;

            // ------- capture every conv_out sample whenever it appears --
            if (out_valid && out_wr_ptr < N_TOTAL) begin
                out_mem[out_wr_ptr] <= {{(RESULT_BYTES*8-OW){conv_out[OW-1]}}, conv_out};
                out_wr_ptr          <= out_wr_ptr + 1'b1;
            end

            case (state)

                // ---- 1) receive 9 signed kernel bytes ------------------
                S_RECV_KERNEL: begin
                    if (rx_valid) begin
                        kernel_mem[kbyte_cnt] <= rx_byte;
                        if (kbyte_cnt == 8) begin
                            kbyte_cnt <= 0;
                            state     <= S_RECV_IMAGE;
                        end else
                            kbyte_cnt <= kbyte_cnt + 1'b1;
                    end
                end

                // ---- 2) receive IMG_W*IMG_H unsigned image bytes -------
                S_RECV_IMAGE: begin
                    if (rx_valid) begin
                        img_mem[pix_cnt[AW-1:0]] <= rx_byte;
                        if (pix_cnt == N_PIX-1) begin
                            pix_cnt <= 0;
                            state   <= S_LOAD_W_0;
                        end else
                            pix_cnt <= pix_cnt + 1'b1;
                    end
                end

                // ---- 3) push the 3 kernel rows into the slice ----------
                // (last row first, per trim_slice_top's loading order)
                S_LOAD_W_0: begin
                    w_load   <= 1'b1;
                    w_row_in <= { {16{kernel_mem[8][7]}}, kernel_mem[8],
                                  {16{kernel_mem[7][7]}}, kernel_mem[7],
                                  {16{kernel_mem[6][7]}}, kernel_mem[6] };
                    state <= S_LOAD_W_1;
                end
                S_LOAD_W_1: begin
                    w_load   <= 1'b1;
                    w_row_in <= { {16{kernel_mem[5][7]}}, kernel_mem[5],
                                  {16{kernel_mem[4][7]}}, kernel_mem[4],
                                  {16{kernel_mem[3][7]}}, kernel_mem[3] };
                    state <= S_LOAD_W_2;
                end
                S_LOAD_W_2: begin
                    w_load   <= 1'b1;
                    w_row_in <= { {16{kernel_mem[2][7]}}, kernel_mem[2],
                                  {16{kernel_mem[1][7]}}, kernel_mem[1],
                                  {16{kernel_mem[0][7]}}, kernel_mem[0] };
                    state <= S_COMPUTE;
                end

                // ---- 4) stream the buffered image through the slice, then
                // PIPE_LATENCY extra dummy (zero) cycles to flush the PE
                // array's psum chain so the last few windows fully emerge --
                S_COMPUTE: begin
                    if (pix_cnt < N_TOTAL) begin
                        in_valid <= 1'b1;
                        i_ext    <= (pix_cnt < N_PIX) ? {16'b0, img_mem[pix_cnt[AW-1:0]]} : {IW{1'b0}};
                        pix_cnt  <= pix_cnt + 1'b1;
                    end
                    // once every input (real + flush) has been driven AND
                    // every result captured, move on (out_wr_ptr lags
                    // pix_cnt by the slice's pipeline latency)
                    if (pix_cnt == N_TOTAL && out_wr_ptr == N_TOTAL) begin
                        pix_cnt <= 0;
                        state   <= S_SEND_RESULT;
                    end
                end

                // ---- 5) send all results back, LSB byte first, then loop
                // back immediately (no blocking dwell -- see done_led's
                // independent pulse-stretcher above) ----
                S_SEND_RESULT: begin
                    if (!tx_busy && !tx_start) begin
                        tx_byte  <= out_mem[send_addr[AW-1:0]][send_byte_idx*8 +: 8];
                        tx_start <= 1'b1;
                        if (send_byte_idx == RESULT_BYTES-1) begin
                            send_byte_idx <= 0;
                            if (send_addr == N_TOTAL-1) begin
                                send_addr      <= 0;
                                out_wr_ptr     <= 0;   // reset for the next transaction
                                txn_done_pulse <= 1'b1;
                                state          <= S_RECV_KERNEL;
                            end else
                                send_addr <= send_addr + 1'b1;
                        end else
                            send_byte_idx <= send_byte_idx + 1'b1;
                    end
                end

                default: state <= S_RECV_KERNEL;
            endcase
        end
    end

endmodule
