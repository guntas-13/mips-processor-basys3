`timescale 1ns / 1ps

module MemReadWrite(
input clk,
input en,
input wen,
input [15:0] addr,
input [31:0] din,
output [31:0] dout);

blk_mem_gen_0 memory(.clka(clk), .ena(en), .wea(wen), .addra(addr), .dina(din), .douta(dout));

endmodule
