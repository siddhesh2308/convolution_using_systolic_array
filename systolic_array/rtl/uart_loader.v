// uart_loader.v - assembles WIDTH-bit words from WIDTH/8 UART bytes
// (MSB first) and routes each word to the correct external buffer port
// on Top_Stage2_MxN.
//
// Expected host byte stream, (WIDTH/8) bytes per word, MSB-first:
//   word 0 .. ROWS*COLS-1            -> weight buffer, addr = word_idx
//                                        (row-major: w[0][0],w[0][1],...)
//   then ROWS groups of ROWS words   -> row buffers 0..ROWS-1,
//                                        addr = position within that row's group
// After the last word, 'start' pulses once and load_complete goes high.
`timescale 1ns/1ps
module UART_Loader #(
    parameter WIDTH     = 24,
    parameter ROWS      = 3,
    parameter COLS      = 3,
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire clk,
    input  wire rst,
    input  wire rx,

    // weight buffer
    output reg                              ext_we_w,
    output reg  [$clog2(ROWS*COLS)-1:0]     ext_addr_w,
    output reg  signed [WIDTH-1:0]          ext_data_w,

    // row buffers - flattened, one slice per row
    output reg  [ROWS-1:0]                  ext_we_row_flat,
    output reg  [ROWS*$clog2(ROWS)-1:0]     ext_addr_row_flat,
    output reg  signed [ROWS*WIDTH-1:0]     ext_data_row_flat,

    output reg                              start,          // 1-cycle pulse once every word is in
    output reg                              load_complete   // stays high after that
);

    localparam WADDR = $clog2(ROWS*COLS);
    localparam RADDR = $clog2(ROWS);
    localparam BYTES_PER_WORD = WIDTH/8;
    localparam TOTAL_WEIGHT_WORDS = ROWS*COLS;
    localparam TOTAL_ACT_WORDS    = ROWS*ROWS;

    // ---- UART byte receiver ----
    wire [7:0] rx_byte;
    wire       rx_valid;

    UART_RX #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) rx_inst (
        .clk(clk), .rst(rst), .rx(rx),
        .data_out(rx_byte), .data_valid(rx_valid)
    );

    // ---- byte -> WIDTH-bit word assembly (MSB-first) ----
    reg [$clog2(BYTES_PER_WORD)-1:0] byte_cnt;
    reg [WIDTH-1:0]                  word_reg;
    reg                              word_ready; // 1-cycle pulse when a full word has landed

    always @(posedge clk) begin
        if (rst) begin
            byte_cnt   <= 0;
            word_reg   <= 0;
            word_ready <= 1'b0;
        end else begin
            word_ready <= 1'b0; // default
            if (rx_valid) begin
                word_reg <= {word_reg[WIDTH-9:0], rx_byte}; // shift in, MSB first
                if (byte_cnt == BYTES_PER_WORD-1) begin
                    byte_cnt   <= 0;
                    word_ready <= 1'b1;
                end else begin
                    byte_cnt <= byte_cnt + 1;
                end
            end
        end
    end

    // ---- word routing ----
    // in_act_phase / weight_idx handle the weight words; once all weight
    // words are in, row_idx/elem_idx (no division needed) route each
    // activation word to its row buffer + position within that row.
    reg                    in_act_phase;
    reg [WADDR-1:0]        weight_idx;
    reg [RADDR-1:0]        row_idx, elem_idx;

    always @(posedge clk) begin
        if (rst) begin
            in_act_phase  <= 1'b0;
            weight_idx    <= 0;
            row_idx       <= 0;
            elem_idx      <= 0;
            start         <= 1'b0;
            load_complete <= 1'b0;

            ext_we_w          <= 1'b0;
            ext_addr_w        <= {WADDR{1'b0}};
            ext_data_w        <= {WIDTH{1'b0}};
            ext_we_row_flat   <= {ROWS{1'b0}};
            ext_addr_row_flat <= {(ROWS*RADDR){1'b0}};
            ext_data_row_flat <= {(ROWS*WIDTH){1'b0}};
        end else begin
            // defaults: every we pulse is exactly one cycle wide
            ext_we_w        <= 1'b0;
            ext_we_row_flat <= {ROWS{1'b0}};
            start           <= 1'b0;

            if (word_ready && !load_complete) begin
                if (!in_act_phase) begin
                    // ---- weight word ----
                    ext_we_w   <= 1'b1;
                    ext_addr_w <= weight_idx;
                    ext_data_w <= word_reg;

                    if (weight_idx == TOTAL_WEIGHT_WORDS-1) begin
                        in_act_phase <= 1'b1;
                        weight_idx   <= 0; // don't-care from here on
                    end else begin
                        weight_idx <= weight_idx + 1;
                    end
                end else begin
                    // ---- activation word: row_idx / elem_idx track position
                    //      without needing a division by ROWS ----
                    ext_we_row_flat[row_idx]                     <= 1'b1;
                    ext_addr_row_flat[row_idx*RADDR +: RADDR]    <= elem_idx;
                    ext_data_row_flat[row_idx*WIDTH +: WIDTH]    <= word_reg;

                    if (elem_idx == ROWS-1) begin
                        elem_idx <= 0;
                        if (row_idx == ROWS-1) begin
                            load_complete <= 1'b1;
                            start         <= 1'b1; // fire Controller's 'start' for one cycle
                        end else begin
                            row_idx <= row_idx + 1;
                        end
                    end else begin
                        elem_idx <= elem_idx + 1;
                    end
                end
            end
        end
    end

endmodule
