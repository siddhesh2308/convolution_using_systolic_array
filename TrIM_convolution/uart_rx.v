// =============================================================
// uart_rx.v : simple 8N1 UART receiver
// =============================================================
module uart_rx #(
    parameter CLKS_PER_BIT = 868   // e.g. 100MHz / 115200 baud ~= 868
)(
    input        clk,
    input        rst_n,
    input        rx_serial,
    output reg       rx_valid,   // pulses 1 cycle when rx_byte is new
    output reg [7:0] rx_byte
);

    localparam IDLE  = 0, START = 1, DATA = 2, STOP = 3;

    reg [1:0]  state      = IDLE;
    reg [15:0] clk_count  = 0;
    reg [2:0]  bit_index  = 0;
    reg [7:0]  shift_reg  = 0;

    // 2-flop synchronizer for the async serial input
    reg rx_sync0 = 1'b1, rx_sync1 = 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync0 <= 1'b1;
            rx_sync1 <= 1'b1;
        end else begin
            rx_sync0 <= rx_serial;
            rx_sync1 <= rx_sync0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            clk_count <= 0;
            bit_index <= 0;
            rx_valid  <= 1'b0;
            rx_byte   <= 8'h00;
        end else begin
            rx_valid <= 1'b0;

            case (state)
                IDLE: begin
                    clk_count <= 0;
                    bit_index <= 0;
                    if (rx_sync1 == 1'b0)          // start bit detected
                        state <= START;
                end

                START: begin
                    // sample mid-bit to confirm it's a real start bit
                    if (clk_count == (CLKS_PER_BIT-1)/2) begin
                        if (rx_sync1 == 1'b0) begin
                            clk_count <= 0;
                            state     <= DATA;
                        end else
                            state <= IDLE;         // false start, glitch
                    end else
                        clk_count <= clk_count + 1'b1;
                end

                DATA: begin
                    if (clk_count < CLKS_PER_BIT-1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count            <= 0;
                        shift_reg[bit_index] <= rx_sync1;
                        if (bit_index < 7) begin
                            bit_index <= bit_index + 1'b1;
                        end else begin
                            bit_index <= 0;
                            state     <= STOP;
                        end
                    end
                end

                STOP: begin
                    if (clk_count < CLKS_PER_BIT-1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 0;
                        state     <= IDLE;
                        rx_byte   <= shift_reg;
                        rx_valid  <= 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
