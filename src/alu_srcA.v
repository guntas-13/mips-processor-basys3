module ALU_SRC_MUX (
    input [31:0] rs,
    input [4:0] shamt,
    input select_shamt,
    output [31:0] alu_srcA
);

    assign alu_srcA = (select_shamt) ? {27'b0, shamt} : rs; // Zero-extend shamt to 32 bits
endmodule
