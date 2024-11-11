`timescale 1ns / 1ps

module jump_tb();
reg clk;
reg en;
reg jump;
reg [31:0] pc;
reg [25:0] addr;
reg [3:0] path_index;
reg [31:0] reg_addr;
wire [31:0] pc_out;
wire jump_done;


JumpModule uut(
.clk(clk),
.en(en),
.jump(jump),
.pc(pc),
.addr(addr),
.path_index(path_index),
.reg_addr(reg_addr),
.pc_out(pc_out),
.jump_done(jump_done)
);

initial begin
    clk <= 1;
    forever #0.5 clk <= ~clk;
end

initial begin
    en <= 0;
    pc <= 32'd3;
    path_index <= 4'd6;
    jump <= 1;
    addr <= 32'd5;
    reg_addr <= 32'd4;
    #5;
    en <= 1;
    #20;
    $finish();
end

endmodule
