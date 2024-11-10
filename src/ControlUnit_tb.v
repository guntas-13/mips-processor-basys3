`timescale 1ns / 1ps

module ControlUnit_tb();
reg clk;
reg top_en;
wire ID;
wire IF;
wire EX;
wire MEM;
wire WB;
wire JU;
wire BR;

initial begin
    clk <= 1;
    forever #0.5 clk = ~clk;
end

ControlUnit uut(
.clk(clk),
.top_en(top_en),
.ID(ID),
.IF(IF),
.EX(EX),
.MEM(MEM),
.WB(WB),
.JU(JU),
.BR(BR)
);

initial begin
    top_en <= 0;
    #5;
    top_en <= 1;
    #20
    $finish();
end
 
endmodule
