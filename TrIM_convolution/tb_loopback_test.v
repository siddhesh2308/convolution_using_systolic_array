`timescale 1ns/1ps
module tb_loopback_test;
    localparam IW=24, K=3, IMG_W=6, IMG_H=6, CLKS_PER_BIT=8;
    localparam OW=2*IW+K+2, RESULT_BYTES=(OW+7)/8, N_PIX=IMG_W*IMG_H;
    localparam PIPE_LATENCY=3, N_TOTAL=N_PIX+PIPE_LATENCY;

    reg clk=0, rst_n=0, tb_tx=1'b1;
    wire dut_tx, done_led;
    always #5 clk=~clk;

    trim_fpga_top #(.IW(IW),.K(K),.IMG_W(IMG_W),.IMG_H(IMG_H),.CLKS_PER_BIT(CLKS_PER_BIT))
        dut(.clk(clk), .rst_n(rst_n), .rx_serial(tb_tx), .tx_serial(dut_tx), .done_led(done_led));

    integer bi, i, r, txn;
    reg [7:0] one_byte;
    task send_byte(input [7:0] b);
        begin
            tb_tx=0; repeat(CLKS_PER_BIT) @(posedge clk);
            for (bi=0;bi<8;bi=bi+1) begin tb_tx=b[bi]; repeat(CLKS_PER_BIT) @(posedge clk); end
            tb_tx=1; repeat(CLKS_PER_BIT) @(posedge clk);
        end
    endtask
    task recv_byte(output [7:0] b);
        integer j;
        begin
            @(negedge dut_tx); repeat(CLKS_PER_BIT/2) @(posedge clk);
            for (j=0;j<8;j=j+1) begin repeat(CLKS_PER_BIT) @(posedge clk); b[j]=dut_tx; end
            repeat(CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    reg signed [7:0] kern[0:8];
    reg [7:0] img[0:N_PIX-1];

    initial begin
        kern[0]=0;kern[1]=1;kern[2]=0;kern[3]=0;kern[4]=0;kern[5]=0;kern[6]=0;kern[7]=0;kern[8]=0;
        for (i=0;i<N_PIX;i=i+1) img[i]=i;

        rst_n=0; repeat(5) @(posedge clk); rst_n=1; repeat(5) @(posedge clk);

        for (txn=0; txn<2; txn=txn+1) begin
            $display("=== Transaction %0d (no reset pulse) ===", txn);
            for (i=0;i<9;i=i+1) send_byte(kern[i]);
            for (i=0;i<N_PIX;i=i+1) send_byte(img[i]);
            for (r=0;r<N_TOTAL*RESULT_BYTES;r=r+1) recv_byte(one_byte);
            $display("Transaction %0d: received all %0d bytes OK", txn, N_TOTAL*RESULT_BYTES);
            // wait for done_led without waiting the full real 5M-cycle dwell
            // (DWELL_CYCLES is baked into the design; just wait for the
            // led to assert then immediately keep clocking through it)
            wait(done_led);
            $display("Transaction %0d: done_led asserted", txn);
        end
        $display("PASS: two back-to-back transactions completed with no manual reset.");
        $finish;
    end
endmodule
