`timescale 1ns / 1ps

module ALU(
    input clk,
    input en,
    input [3:0] alu_control,
    input [31:0] alu_srcA,          // this corresponds to $rs or shamt
    input [31:0] alu_srcB,          // this corresponds to $rt
    output reg [31:0] alu_result,   // this corresponds to $rd
    output reg [31:0] hi,
    output reg [31:0] lo,
    output reg overflow,
    output reg alu_done,
    output alu_zero
);

    parameter ADD  = 4'b0000;
    parameter SUB  = 4'b0001;
    parameter AND  = 4'b0010;
    parameter OR   = 4'b0011;
    parameter NOR  = 4'b0100;
    parameter SLT  = 4'b0101;
    parameter SLL  = 4'b0110;
    parameter SRL  = 4'b0111;
    parameter MULT = 4'b1000;
    parameter DIV  = 4'b1001;

// Modification1: No need of these extra registers. MULT and DIV work fine as coded now.
//    reg [63:0] mult_result; 
//    reg [31:0] div_quotient;
//    reg [31:0] div_remainder;

    assign alu_zero = (alu_result == 32'd0);

    initial begin
        hi = 32'd0;
        lo = 32'd0;
        overflow = 1'b0;
        alu_done = 1'b0;
    end

    always @(posedge clk) begin
        if (en) begin
            // alu_done <= 1'b0;
            overflow <= 1'b0;

            case (alu_control)
                ADD: begin
                    {overflow, alu_result} <= alu_srcA + alu_srcB;
                    alu_done <= 1'b1;
                end
                SUB: begin
                    {overflow, alu_result} <= alu_srcA - alu_srcB;
                    alu_done <= 1'b1;
                end
                AND: begin
                    alu_result <= alu_srcA & alu_srcB;
                    alu_done <= 1'b1;
                end
                OR: begin
                    alu_result <= alu_srcA | alu_srcB;
                    alu_done <= 1'b1;
                end
                NOR: begin
                    alu_result <= ~(alu_srcA | alu_srcB);
                    alu_done <= 1'b1;
                end
                SLT: begin
                    alu_result <= (alu_srcA < alu_srcB) ? 32'd1 : 32'd0;
                    alu_done <= 1'b1;
                end
                SLL: begin
                    alu_result <= alu_srcB << alu_srcA[4:0]; // Shift amount in alu_srcA
                    alu_done <= 1'b1;
                end
                SRL: begin
                    alu_result <= alu_srcB >> alu_srcA[4:0]; // Shift amount in alu_srcA
                    alu_done <= 1'b1;
                end
                MULT: begin
                    {hi, lo} = alu_srcA * alu_srcB;
//                    hi <= mult_result[63:32]; Modification 1: Removed unnecessary regs
//                    lo <= mult_result[31:0]; Modification 1: Removed unnecessary regs
                    alu_done <= 1'b1;
                end
                DIV: begin
                    if (alu_srcB != 0) begin
                        lo = alu_srcA / alu_srcB; // Modification 1: Removed unnecessary regs
                        hi = alu_srcA % alu_srcB; // Modification 1: Removed unnecessary regs
                        alu_done <= 1'b1;
                    end else begin
                        overflow <= 1'b1; // Set overflow on divide-by-zero
                        lo <= 32'd0;
                        hi <= 32'd0;
                        alu_done <= 1'b1;
                    end
                end
                default: begin
                    alu_result <= 32'd0;
                    alu_done <= 1'b1;
                end
            endcase
        end
         else begin
         alu_done <= 1'b0; // Modification 2: Added condition. Now, alu_done == 1 iff en = 1 and op is done! 
         end
    end
endmodule