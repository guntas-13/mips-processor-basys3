`timescale 1ns / 1ps

module ControlUnit_tb();
reg clk;
reg top_en;
reg infer;
reg [9:0] infer_addr;
wire [31:0] infer_data;
wire ID;
wire IF;
wire REG;
wire EX;
wire MEM;
wire WB;
wire JU;
wire BR;
wire SK;
wire [6:0] LED_out;
wire [3:0] Anode_Activate;

initial begin
    clk <= 1;
    forever #0.5 clk = ~clk;
end

ControlUnit uut(
.fast_clk(clk),
.top_en(top_en),
.infer(infer),
.infer_addr(infer_addr),
.infer_data(infer_data),
.ID(ID),
.IF(IF),
.REG(REG),
.EX(EX),
.MEM(MEM),
.WB(WB),
.JU(JU),
.BR(BR),
.SK(SK),
.LED_out(LED_out),
.Anode_Activate(Anode_Activate)
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
