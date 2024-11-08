`timescale 1ns/1ps

module MUX8x3_RegWrite(
    input [31:0] alu_result,
    input [31:0] dout,
    input [31:0] hi,
    input [31:0] lo,
    input [31:0] PC_updated,   // For jal instruction need PC + 4 to be written to register file
    input SEL0_MemtoReg0,
    input SEL1_MemtoReg1,
    input SEL2_Jump,            // For jal instruction need PC + 4 to be written to register file
    output reg [31:0] mux_out
);

    always @(*) begin
        case ({SEL2_Jump, SEL1_MemtoReg1, SEL0_MemtoReg0})
            3'b000: mux_out = alu_result;
            3'b001: mux_out = dout;
            3'b010: mux_out = hi;
            3'b011: mux_out = lo;
            3'b100: mux_out = PC_updated;
            default: mux_out = 32'bx;
        endcase
    end

endmodule