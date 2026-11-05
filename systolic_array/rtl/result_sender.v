// result_sender.v - on a 'start' pulse (feed it Top_Stage2_MxN's 'done'
// output directly, it's already a 1-cycle pulse), walks addr 0..ROWS-1
// across all COLS output column buffers and streams the ROWS*COLS values
// out over UART, then appends a 4-byte cycle count for timing measurement.
//
// Output byte stream (ROWS*COLS*(WIDTH/8) + 4 bytes total):
//   addr 0: col0[0], col1[0], ..., col(COLS-1)[0]   (WIDTH/8 bytes each, MSB-first)
//   addr 1: col0[1], col1[1], ..., col(COLS-1)[1]
//   ...
//   addr ROWS-1: col0[ROWS-1], ..., col(COLS-1)[ROWS-1]
//   cycle_count_in, 4 bytes, MSB-first (unsigned)
`timescale 1ns/1ps
module Result_Sender #(
    parameter WIDTH     = 24,
    parameter ROWS      = 2,
    parameter COLS      = 2,
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire clk,
    input  wire rst,
    input  wire start, // pulse to kick off readback (e.g. accelerator 'done')

    // combinational connection to the accelerator's registered col outputs
    input  wire signed [COLS*WIDTH-1:0] col_in_flat,
    output reg  [$clog2(ROWS)-1:0]      addr_out, // drives ext_addr_col (same address, all COLS)

    input  wire [31:0] cycle_count_in, // latched start->done cycle count

    output wire tx,
    output reg  busy // high for the whole readback sequence
);

    localparam RADDR           = $clog2(ROWS);
    localparam CADDR           = (COLS > 1) ? $clog2(COLS) : 1;
    localparam BYTES_PER_WORD  = WIDTH/8;
    localparam BIDX_BITS       = (BYTES_PER_WORD > 1) ? $clog2(BYTES_PER_WORD) : 1;

    localparam S_IDLE     = 4'd0,
               S_READWAIT = 4'd1,
               S_LOAD     = 4'd2,
               S_SEND     = 4'd3,
               S_WAITBUSY = 4'd4,
               S_WAITIDLE = 4'd5,
               S_NEXTADDR = 4'd6,
               S_TLOAD    = 4'd7,
               S_TSEND    = 4'd8,
               S_TWAITBUSY= 4'd9,
               S_TWAITIDLE= 4'd10,
               S_DONE     = 4'd11;

    reg [3:0]              state;
    reg [RADDR-1:0]        cur_addr;
    reg [CADDR-1:0]        col_cnt;    // which column (0..COLS-1) we're sending
    reg [BIDX_BITS-1:0]    byte_idx;   // which byte within that column's word (0=MSB)
    reg [COLS*WIDTH-1:0]   bundle;     // raw capture of col_in_flat, indexed directly (never shifted)

    reg [1:0]              tbyte_cnt;        // 0..3 (4 bytes of cycle count)
    reg [31:0]             tbundle;

    reg                    tx_start;
    reg  [7:0]             tx_data;
    wire                   tx_busy;

    UART_TX #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) tx_inst (
        .clk(clk), .rst(rst),
        .tx_start(tx_start), .data_in(tx_data),
        .tx(tx), .busy(tx_busy)
    );

    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            addr_out  <= {RADDR{1'b0}};
            cur_addr  <= {RADDR{1'b0}};
            col_cnt   <= {CADDR{1'b0}};
            byte_idx  <= {BIDX_BITS{1'b0}};
            bundle    <= {(COLS*WIDTH){1'b0}};
            tbyte_cnt <= 2'd0;
            tbundle   <= 32'd0;
            tx_start  <= 1'b0;
            busy      <= 1'b0;
        end else begin
            tx_start <= 1'b0; // default; pulsed explicitly below

            case (state)
                S_IDLE: begin
                    if (start) begin
                        cur_addr <= {RADDR{1'b0}};
                        addr_out <= {RADDR{1'b0}};
                        busy     <= 1'b1;
                        state    <= S_READWAIT;
                    end
                end

                // one cycle for the col buffers' synchronous read to catch up
                // to the address we just drove
                S_READWAIT: begin
                    state <= S_LOAD;
                end

                S_LOAD: begin
                    bundle   <= col_in_flat;
                    col_cnt  <= {CADDR{1'b0}};
                    byte_idx <= {BIDX_BITS{1'b0}};
                    state    <= S_SEND;
                end

                // column c's word sits at bundle[c*WIDTH +: WIDTH]; byte_idx=0
                // is that word's MOST significant byte (MSB-first on the wire),
                // so col0's word goes out fully before col1's starts, matching
                // the documented "col0[a], col1[a], ..." order.
                S_SEND: begin
                    tx_data  <= bundle[col_cnt*WIDTH + (BYTES_PER_WORD-1-byte_idx)*8 +: 8];
                    tx_start <= 1'b1;
                    state    <= S_WAITBUSY;
                end

                // wait for UART_TX to actually latch the start and go busy
                S_WAITBUSY: begin
                    if (tx_busy) state <= S_WAITIDLE;
                end

                // wait for the byte to finish transmitting
                S_WAITIDLE: begin
                    if (!tx_busy) begin
                        if (byte_idx == BYTES_PER_WORD-1) begin
                            byte_idx <= {BIDX_BITS{1'b0}};
                            if (col_cnt == COLS-1) begin
                                state <= S_NEXTADDR;
                            end else begin
                                col_cnt <= col_cnt + 1'b1;
                                state   <= S_SEND;
                            end
                        end else begin
                            byte_idx <= byte_idx + 1'b1;
                            state    <= S_SEND;
                        end
                    end
                end

                S_NEXTADDR: begin
                    if (cur_addr == ROWS-1) begin
                        state <= S_TLOAD; // done with all addresses -> send timing
                    end else begin
                        cur_addr <= cur_addr + 1'b1;
                        addr_out <= cur_addr + 1'b1;
                        state    <= S_READWAIT;
                    end
                end

                // ---- append the 4-byte cycle count ----
                S_TLOAD: begin
                    tbundle   <= cycle_count_in;
                    tbyte_cnt <= 2'd0;
                    state     <= S_TSEND;
                end

                S_TSEND: begin
                    tx_data  <= tbundle[31 -: 8];
                    tx_start <= 1'b1;
                    state    <= S_TWAITBUSY;
                end

                S_TWAITBUSY: begin
                    if (tx_busy) state <= S_TWAITIDLE;
                end

                S_TWAITIDLE: begin
                    if (!tx_busy) begin
                        tbundle <= tbundle << 8;
                        if (tbyte_cnt == 2'd3) begin
                            state <= S_DONE;
                        end else begin
                            tbyte_cnt <= tbyte_cnt + 1'b1;
                            state     <= S_TSEND;
                        end
                    end
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule