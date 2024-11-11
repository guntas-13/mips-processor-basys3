`timescale 1ns / 1ps

module ControlUnit_tb();
reg clk;
reg top_en;
reg infer;
reg [9:0] infer_addr;
wire ID;
wire IF;
wire REG;
wire EX;
wire MEM;
wire WB;
wire JU;
wire BR;
wire SK;

initial begin
    clk <= 1;
    forever #0.5 clk = ~clk;
end

ControlUnit uut(
.fast_clk(clk),
.top_en(top_en),
.infer(infer),
.infer_addr(infer_addr),
.ID(ID),
.IF(IF),
.REG(REG),
.EX(EX),
.MEM(MEM),
.WB(WB),
.JU(JU),
.BR(BR),
.SK(SK)
);

initial begin
    top_en <= 0;
    infer <= 0;
    #5;
    top_en <= 1;
    #990;
    infer <= 1;
    infer_addr <= 1;
    #10;
    $finish();
end
 
endmodule
