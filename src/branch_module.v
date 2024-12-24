`timescale 1ns / 1ps

module BranchModule(
input en,
input alu_zero,
input [31:0] imm,
input [31:0] pc,
output reg [31:0] pc_out
);

always @ (posedge en) begin
    pc_out <= (alu_zero)? pc + imm: pc;
end

endmodule