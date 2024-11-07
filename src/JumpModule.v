`timescale 1ns / 1ps


module JumpModule(
input en,
input jump,
input [31:0] pc,
input [25:0] addr,
input [3:0] path_index,
input [31:0] reg_addr,
output [31:0] pc_out
);

assign pc_out = (jump & 

endmodule
