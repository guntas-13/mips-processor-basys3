`timescale 1ns / 1ps

module MemRW_tb();

reg clk;
initial begin
    clk = 1;
    forever #0.5 clk = ~clk;
end

reg en;
reg wen;
reg [15:0] addr;
reg [31:0] din;
wire [31:0] dout;

MemReadWrite uut(
.clk(clk),
.en(en),
.wen(wen),
.addr(addr),
.din(din),
.dout(dout));

initial begin
    en = 0;
    wen = 0;
    addr = 32'd0;
    din = 32'd0;
    #5;
    // Read addr 1
    en = 1;
    wen = 0;
    addr = 32'd1;
    din = 32'd2;
    #5;
    en = 0;
    #5;
    // Read addr 2
    en = 1;
    wen = 0;
    addr = 32'd2;
    din = 32'd3;
    #5;
    en = 0;
    #5;
    // Read addr 1
    en = 1;
    wen = 0;
    addr = 32'd1;
    din = 32'd2;
    #5;
    en = 0;
    wen = 1;
    addr = 32'd5;
    din = 32'd4;
    #5;
    //Read addr 5
    en = 1;
    wen = 0;
    addr = 32'd5;
    din = 32'd3;
    #5;
    en = 0;
    #5;
    // Read addr 51199
    en = 1;
    wen = 0;
    addr = 32'd51199;
    din = 32'd0;
    #5;
    en = 0;
    #5;
    // Read addr 51201
    en = 1;
    wen = 0;
    addr = 32'd51201;
    din = 32'd1;
    #5;
    en = 0;
    #5;
    // Write at addr 0
    en = 1;
    wen = 1;
    addr = 32'd0;
    din = 32'd1;
    #1;
    en = 0;
    wen = 0;
    #1
    // Read addr 0
    en = 1;
    wen = 0;
    addr = 32'd0;
    din = 32'd2;
    #5;
    $finish;
end

endmodule
