`timescale 1ns / 1ps

module BranchModule(
input clk,
input en,
input branch,
input alu_zero,
input [31:0] imm,
input [31:0] pc,
output [31:0] pc_out
);

assign pc_out = (branch & en & alu_zero)? pc + (imm << 2): pc;

endmodule
