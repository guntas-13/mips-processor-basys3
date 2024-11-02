`timescale 1ns/1ps

module MUX2x1_tb;

    parameter WIDTH_5 = 5;
    parameter WIDTH_32 = 32;

    reg [WIDTH_5-1:0] input0_5bit;
    reg [WIDTH_5-1:0] input1_5bit;
    reg sel_5bit;
    wire [WIDTH_5-1:0] mux_out_5bit;

    reg [WIDTH_32-1:0] input0_32bit;
    reg [WIDTH_32-1:0] input1_32bit;
    reg sel_32bit;
    wire [WIDTH_32-1:0] mux_out_32bit;

    MUX2x1 #(WIDTH_5) mux5bit (
        .input0(input0_5bit),
        .input1(input1_5bit),
        .SEL(sel_5bit),
        .mux_out(mux_out_5bit)
    );

    MUX2x1 #(WIDTH_32) mux32bit (
        .input0(input0_32bit),
        .input1(input1_32bit),
        .SEL(sel_32bit),
        .mux_out(mux_out_32bit)
    );

    initial begin
        // Test 5-bit MUX
        $display("Testing 5-bit MUX...");
        input0_5bit = 5'b00001;
        input1_5bit = 5'b11111;

        sel_5bit = 1'b0; #5;
        $display("SEL=0, mux_out_5bit=%b (Expected: 00001)", mux_out_5bit);

        sel_5bit = 1'b1; #5;
        $display("SEL=1, mux_out_5bit=%b (Expected: 11111)", mux_out_5bit);

        sel_5bit = 1'bx; #5;
        $display("SEL=X, mux_out_5bit=%b (Expected: XXXXX)", mux_out_5bit);

        // Test 32-bit MUX
        $display("\nTesting 32-bit MUX...");
        input0_32bit = 32'h0001;
        input1_32bit = 32'hFFFF;

        sel_32bit = 1'b0; #5;
        $display("SEL=0, mux_out_32bit=%h (Expected: 00000001)", mux_out_32bit);

        sel_32bit = 1'b1; #5;
        $display("SEL=1, mux_out_32bit=%h (Expected: 0000FFFF)", mux_out_32bit);

        sel_32bit = 1'bx; #5;
        $display("SEL=X, mux_out_32bit=%h (Expected: XXXXXXXX)", mux_out_32bit);
        $finish;
    end

endmodule
