`timescale 1ns / 1ps

module ALU_tb;
    reg clk;
    reg en;
    reg [3:0] alu_control;
    reg [31:0] alu_srcA;
    reg [31:0] alu_srcB;

    wire [31:0] alu_result;
    wire [31:0] hi;
    wire [31:0] lo;
    wire overflow;
    wire alu_done;
    wire alu_zero;

    ALU uut (
        .clk(clk),
        .en(en),
        .alu_control(alu_control),
        .alu_srcA(alu_srcA),
        .alu_srcB(alu_srcB),
        .alu_result(alu_result),
        .hi(hi),
        .lo(lo),
        .overflow(overflow),
        .alu_done(alu_done),
        .alu_zero(alu_zero)
    );

    initial begin
        clk = 1;
        forever #0.5 clk = ~clk;  // 1ns period (1000MHz clock)
    end

    initial begin
        // Initialize inputs
        en = 0;
        alu_control = 4'b0000;
        alu_srcA = 0;
        alu_srcB = 0;

        // Wait for global reset
        #5;

        // Enable the ALU
        en = 1;

        // Test ADD operation
        alu_control = 4'b0000; // ADD
        alu_srcA = 32'd15;
        alu_srcB = 32'd10;
        #5;
        $display($time, ": ADD: %d + %d = %d", alu_srcA, alu_srcB, alu_result);
        
        en = 0;
        alu_srcA = 32'd7;
        alu_srcB = 32'd6;
        #5;
        en = 1;
        
        // Test SUB operation
        alu_control = 4'b0001; // SUB
        alu_srcA = 32'd25;
        alu_srcB = 32'd10;
        #5;
        $display($time, ": SUB: %d - %d = %d", alu_srcA, alu_srcB, alu_result);

        // Test AND operation
        alu_control = 4'b0010; // AND
        alu_srcA = 32'b1100;
        alu_srcB = 32'b1010;
        #5;
        $display($time, ": AND: %b & %b = %b", alu_srcA, alu_srcB, alu_result);

        // Test OR operation
        alu_control = 4'b0011; // OR
        alu_srcA = 32'b1100;
        alu_srcB = 32'b1010;
        #5;
        $display($time, ": OR: %b | %b = %b", alu_srcA, alu_srcB, alu_result);

        // Test NOR operation
        alu_control = 4'b0100; // NOR
        alu_srcA = 32'b1100;
        alu_srcB = 32'b1010;
        #5;
        $display($time, ": NOR: ~(%b | %b) = %b", alu_srcA, alu_srcB, alu_result);

        // Test SLT operation
        alu_control = 4'b0101; // SLT
        alu_srcA = 32'd5;
        alu_srcB = 32'd10;
        #5;
        $display($time, ": SLT: %d < %d = %d", alu_srcA, alu_srcB, alu_result);

        // Test SLL operation
        alu_control = 4'b0110; // SLL
        alu_srcA = 32'd2;      // shamt (number of shifts)
        alu_srcB = 32'd4;
        #5;
        $display($time, ": SLL: %d << %d = %d", alu_srcB, alu_srcA, alu_result);

        // Test SRL operation
        alu_control = 4'b0111; // SRL
        alu_srcA = 32'd2;      // shamt (number of shifts)
        alu_srcB = 32'd16;
        #5;
        $display($time, ": SRL: %d >> %d = %d", alu_srcB, alu_srcA, alu_result);

        // Test MULT operation
        alu_control = 4'b1000; // MULT
        alu_srcA = 32'd131072;
        alu_srcB = 32'd131072;
        #5;
        $display($time, ": MULT: %d * %d = %d (hi: %d, lo: %d)", alu_srcA, alu_srcB, {hi, lo}, hi, lo);

        // Test DIV operation
        alu_control = 4'b1001; // DIV
        alu_srcA = 32'd20;
        alu_srcB = 32'd3;
        #5;
        $display($time, ": DIV: %d / %d = %d (quotient: %d, remainder: %d)", alu_srcA, alu_srcB, lo, lo, hi);

        // End the test
        en = 0;
        #5;

        $finish;
    end

endmodule