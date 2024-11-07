`timescale 1ns / 1ps

module RegisterFile_tb;

    // Inputs
    reg clk;
    reg en;
    reg reg_write;
    reg [4:0] read_reg1;
    reg [4:0] read_reg2;
    reg [4:0] write_reg;
    reg [31:0] write_data;

    wire [31:0] read_data1;
    wire [31:0] read_data2;
    wire register_done;

    RegisterFile uut (
        .clk(clk),
        .en(en),
        .reg_write(reg_write),
        .read_reg1(read_reg1),
        .read_reg2(read_reg2),
        .write_reg(write_reg),
        .write_data(write_data),
        .read_data1(read_data1),
        .read_data2(read_data2),
        .register_done(register_done)
    );

    initial begin
        clk = 1;
        forever #0.5 clk = ~clk;  // 2ns period (500 MHz)
    end

    initial begin
        en = 0;
        reg_write = 0;
        read_reg1 = 5'd0;
        read_reg2 = 5'd0;
        write_reg = 5'd0;
        write_data = 32'd0;

        #5;
        en = 1;

        // Test 1: Write 100 to register 1
        #5;
        reg_write = 1;
        write_reg = 5'd1;
        write_data = 32'd100;
        #1;
        reg_write = 0;
        read_reg1 = 5'd1;
        #4;
        en = 0;
//        @(posedge register_done);

        // Test 2: Read from register 1
        read_reg1 = 5'd1;
        #5;
//        @(posedge clk);  // Wait for next clock to capture read
//        $display($time, " Read Data1 (should be 100): %d", read_data1);

        // Test 3: Write 200 to register 2
        reg_write = 1;
        write_reg = 5'd2;
        write_data = 32'd200;
        #5;
        reg_write = 0;

        // Wait for register_done signal
//        @(posedge register_done);

        // Test 4: Read from registers 1 and 2
        read_reg1 = 5'd1;
        read_reg2 = 5'd2;
        #5;
//        @(posedge clk);  // Wait for next clock to capture read
        $display($time, " Read Data1 (should be 100): %d", read_data1);
        $display($time, " Read Data2 (should be 200): %d", read_data2);

        // Test 5: Try writing to register 0 (should not change)
        reg_write = 1;
        write_reg = 5'd1;
        write_data = 32'd50;
        #5;
        reg_write = 0;

        // Wait for register_done signal
//        @(posedge register_done);

        // Test 6: Read from register 0
        read_reg1 = 5'd0;
        #5;
//        @(posedge clk);
        $display($time, " Read Data1 (should be 0): %d", read_data1);

        $finish;
    end
endmodule