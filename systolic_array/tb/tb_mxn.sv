`timescale 1ns/1ps
// tb_top_fpga.sv - drives Top_FPGA the way a real host (send_data.py) does:
// bit-bangs UART frames onto uart_rx_pin, decodes uart_tx_pin the same way,
// and checks the result against a software matmul.

module tb_top_fpga;
    parameter WIDTH     = 24;
    parameter ROWS      = 3;
    parameter COLS      = 3;
    parameter CLK_FREQ  = 100_000_000;
    parameter BAUD_RATE = 115_200;

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    reg clk = 0;
    reg rst = 1;
    reg uart_rx_pin = 1;   // UART idles high
    wire uart_tx_pin;
    wire done, sending;

    always #5 clk = ~clk;  // 100 MHz sim clock, matches CLK_FREQ

    Top_FPGA #(
        .WIDTH(WIDTH), .ROWS(ROWS), .COLS(COLS),
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)
    ) dut (
        .clk(clk), .rst(rst),
        .uart_rx_pin(uart_rx_pin), .uart_tx_pin(uart_tx_pin),
        .done(done), .sending(sending)
    );

    // ---- host-side UART bit-bang (mirrors what send_data.py does over real wire) ----
    task send_uart_byte(input [7:0] b);
        integer i;
        begin
            uart_rx_pin = 0;                       // start bit
            repeat (CLKS_PER_BIT) @(negedge clk);
            for (i = 0; i < 8; i = i + 1) begin     // 8 data bits, LSB first (8N1)
                uart_rx_pin = b[i];
                repeat (CLKS_PER_BIT) @(negedge clk);
            end
            uart_rx_pin = 1;                       // stop bit
            repeat (CLKS_PER_BIT) @(negedge clk);
        end
    endtask

    task recv_uart_byte(output [7:0] b);
        integer i;
        begin
            @(negedge uart_tx_pin);                        // start bit begins
            repeat (CLKS_PER_BIT + CLKS_PER_BIT/2) @(negedge clk); // skip to mid of bit0
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = uart_tx_pin;
                repeat (CLKS_PER_BIT) @(negedge clk);
            end
        end
    endtask

    task send_word24(input signed [23:0] val);
        begin
            send_uart_byte(val[23:16]);
            send_uart_byte(val[15:8]);
            send_uart_byte(val[7:0]);
        end
    endtask

    function signed [23:0] bytes_to_word24(input [7:0] b0, input [7:0] b1, input [7:0] b2);
        bytes_to_word24 = {b0, b1, b2};
    endfunction

    integer r, c, i;
    integer si, ri;   // separate loop counters for the concurrent send/recv fork branches
    reg signed [WIDTH-1:0] W   [0:ROWS*COLS-1];   // flattened, row-major
    reg signed [WIDTH-1:0] A   [0:ROWS*ROWS-1];   // flattened, row-major
    reg signed [WIDTH-1:0] EXP [0:ROWS*COLS-1];   // flattened, row-major

    reg [7:0] rb0, rb1, rb2, rb3;
    reg signed [WIDTH-1:0] got [0:ROWS*COLS-1];
    integer errors = 0;

    initial begin
        // ---- build test data ----
        for (r = 0; r < ROWS; r = r + 1)
            for (c = 0; c < COLS; c = c + 1)
                W[r*COLS+c] = (r+1)*10 + (c+1);
        for (r = 0; r < ROWS; r = r + 1)
            for (c = 0; c < ROWS; c = c + 1)
                A[r*ROWS+c] = (r+c+1);              // dense, non-identity
        for (r = 0; r < ROWS; r = r + 1)
            for (c = 0; c < COLS; c = c + 1) begin
                EXP[r*COLS+c] = 0;
                for (i = 0; i < ROWS; i = i + 1)
                    EXP[r*COLS+c] = EXP[r*COLS+c] + A[r*ROWS+i]*W[i*COLS+c];
            end

        repeat (10) @(negedge clk);
        rst = 0;
        repeat (10) @(negedge clk);

        // ---- send and receive concurrently (mirrors real hardware: the
        // device starts replying almost as soon as it gets the last
        // activation byte, which can be before a blocking sender even
        // finishes its own trailing stop-bit bookkeeping) ----
        fork
            begin : SEND_PROC
                for (si = 0; si < ROWS*COLS; si = si + 1)
                    send_word24(W[si]);
                for (si = 0; si < ROWS*ROWS; si = si + 1)
                    send_word24(A[si]);
                $display("[%0t] all %0d words sent", $time, ROWS*COLS+ROWS*ROWS);
            end
            begin : RECV_PROC
                for (ri = 0; ri < ROWS*COLS; ri = ri + 1) begin
                    recv_uart_byte(rb0);
                    recv_uart_byte(rb1);
                    recv_uart_byte(rb2);
                    got[ri] = bytes_to_word24(rb0, rb1, rb2);
                end
                recv_uart_byte(rb0); recv_uart_byte(rb1); recv_uart_byte(rb2); recv_uart_byte(rb3);
                $display("[%0t] all results received, cycle count (start->done): %0d",
                          $time, {rb0, rb1, rb2, rb3});
            end
        join

        // ---- check ----
        for (r = 0; r < ROWS; r = r + 1)
            for (c = 0; c < COLS; c = c + 1) begin
                if (got[r*COLS+c] !== EXP[r*COLS+c]) begin
                    $display("MISMATCH row=%0d col=%0d got=%0d exp=%0d",
                              r, c, got[r*COLS+c], EXP[r*COLS+c]);
                    errors = errors + 1;
                end
            end

        if (errors == 0)
            $display("PASS: full UART round-trip through Top_FPGA correct (ROWS=%0d COLS=%0d)", ROWS, COLS);
        else
            $display("FAIL: %0d mismatches", errors);

        $finish;
    end

    // safety timeout
    initial begin
        #50_000_000;
        $display("TIMEOUT - done never completed / UART link stalled");
        $finish;
    end
endmodule