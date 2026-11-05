// Controller.v - top-level FSM (Stage 1 skeleton)
// States: IDLE -> LOAD -> COMPUTE -> STORE -> DONE -> IDLE
`timescale 1ns/1ps
module Controller (
    input  wire clk,
    input  wire rst,
    input  wire start,          // begin a new operation
    input  wire load_done,      // buffers finished loading
    input  wire compute_done,   // systolic array finished computing
    input  wire store_done,     // output buffer finished storing
    output reg  load_en,
    output reg  compute_en,
    output reg  store_en,
    output reg  done
);

    localparam IDLE    = 3'd0,
               LOAD     = 3'd1,
               COMPUTE  = 3'd2,
               STORE    = 3'd3,
               DONE     = 3'd4;

    reg [2:0] state, next_state;

    always @(posedge clk) begin
        if (rst) state <= IDLE;
        else     state <= next_state;
    end

    always @(*) begin
        next_state = state;
        load_en    = 1'b0;
        compute_en = 1'b0;
        store_en   = 1'b0;
        done       = 1'b0;

        case (state)
            IDLE: begin
                if (start) next_state = LOAD;
            end
            LOAD: begin
                load_en = 1'b1;
                if (load_done) next_state = COMPUTE;
            end
            COMPUTE: begin
                compute_en = 1'b1;
                if (compute_done) next_state = STORE;
            end
            STORE: begin
                store_en = 1'b1;
                if (store_done) next_state = DONE;
            end
            DONE: begin
                done = 1'b1;
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

endmodule
