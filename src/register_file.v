`timescale 1ns/1ps

module RegisterFile(
    input clk,
    input en,
    input reg_write,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input [4:0] write_reg,
    input [31:0] write_data,
    output reg [31:0] read_data1,
    output reg [31:0] read_data2,
    output reg register_done
);

    reg [31:0] registers [0:31];
    integer i;

    initial begin
        register_done = 1'b0;
    end

    always @(posedge clk) begin
        if (en) begin
            register_done <= 1'b0;

            read_data1 <= registers[read_reg1];
            read_data2 <= registers[read_reg2];

            if (reg_write && write_reg != 5'd0) begin
                registers[write_reg] <= write_data;
            end

            register_done <= 1'b1;
        end
         else begin
             register_done <= 1'b0;
         end
    end
endmodule