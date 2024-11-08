`timescale 1ns / 1ps

module BranchModule(
input en,
input branch,
input alu_zero,
input [31:0] imm,
input [31:0] pc,
output reg [31:0] pc_out,
output reg branch_done
);

initial begin
    branch_done = 1'b0;
end
always @ (posedge en) begin
    pc_out <= (branch & en & alu_zero)? pc + imm: pc;
    branch_done <= 1'b1;
end

endmodule