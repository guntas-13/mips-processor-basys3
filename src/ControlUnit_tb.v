`timescale 1ns / 1ps

module ControlUnit_tb();
reg clk;
reg top_en;
reg infer;
reg [15:0] infer_addr;
wire ID;
wire IF;
wire REG;
wire EX;
wire MEM;
wire WB;
wire JU;
wire BR;
wire SK;
wire infer_data;

initial begin
    clk <= 1;
    forever #0.5 clk = ~clk;
end

ControlUnit uut(
.clk(clk),
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
.SK(SK),
.infer_data(infer_data)
);

initial begin
    top_en <= 0;
    #5;
    top_en <= 1;
    #40;
    infer <= 1;
    infer_addr <= 15'd6301;
    #10;
    $finish();
end
 
endmodule
