`timescale 1ns / 1ps

module ALU(
    input en,
    input [3:0] alu_control,
    input [31:0] alu_srcA,          // this corresponds to $rs or shamt
    input [31:0] alu_srcB,          // this corresponds to $rt
    output reg [31:0] alu_result,   // this corresponds to $rd
    output reg [31:0] hi,
    output reg [31:0] lo,
    output reg overflow,
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

    assign alu_zero = (alu_result == 32'd0);

    initial begin
        hi = 32'd0;
        lo = 32'd0;
        overflow = 1'b0;
    end

    always @(posedge en) begin
        overflow <= 1'b0;
        case (alu_control)
            ADD: begin
                {overflow, alu_result} <= alu_srcA + alu_srcB;
            end
            SUB: begin
                {overflow, alu_result} <= alu_srcA - alu_srcB;
            end
            AND: begin
                alu_result <= alu_srcA & alu_srcB;
            end
            OR: begin
                alu_result <= alu_srcA | alu_srcB;
            end
            NOR: begin
                alu_result <= ~(alu_srcA | alu_srcB);
            end
            SLT: begin
                alu_result <= (alu_srcA < alu_srcB) ? 32'd1 : 32'd0;
            end
            SLL: begin
                alu_result <= alu_srcB << alu_srcA[4:0]; // Shift amount in alu_srcA
            end
            SRL: begin
                alu_result <= alu_srcB >> alu_srcA[4:0]; // Shift amount in alu_srcA
            end
            MULT: begin
                {hi, lo} = alu_srcA * alu_srcB;
            end
            DIV: begin
                if (alu_srcB != 0) begin
                    lo = alu_srcA / alu_srcB;
                    hi = alu_srcA % alu_srcB;
                end else begin
                    overflow <= 1'b1; // Set overflow on divide-by-zero
                    lo <= 32'd0;
                    hi <= 32'd0;
                end
            end
            default: begin
                alu_result <= 32'd0;
            end
        endcase
    end
endmodule