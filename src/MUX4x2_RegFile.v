`timescale 1ns/1ps

module MUX4x2_RegFile(
    input [4:0] rt,
    input [4:0] rd,
    input [4:0] fix_ra,
    input SEL0_RegDst,
    input SEL1_Jump,
    output reg [4:0] mux_out
);

    always @(*) begin
        case ({SEL1_Jump, SEL0_RegDst})
            2'b00: mux_out = rt;        // For I-Types RegDst = 0
            2'b01: mux_out = rd;        // For R-Types RegDst = 1
            2'b10: mux_out = fix_ra;    // For J-Types jal R[31] ($ra) = PC + 4
            default: mux_out = 5'bx;
        endcase
    end
    
endmodule