`timescale 1ns/1ps

module MUX2x1 #(parameter WIDTH = 32) (
    input [WIDTH-1:0] input0,
    input [WIDTH-1:0] input1,
    input SEL,
    output reg [WIDTH-1:0] mux_out
);

    always @(*) begin
        case (SEL)
            1'b0: mux_out = input0;
            1'b1: mux_out = input1;
            default: mux_out = 32'bx;
        endcase
    end
endmodule
