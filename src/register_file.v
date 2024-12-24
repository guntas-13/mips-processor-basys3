`timescale 1ns/1ps

module RegisterFile(
    input en,
    input reg_write,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input [4:0] write_reg,
    input [31:0] write_data,
    output reg [31:0] read_data1,
    output reg [31:0] read_data2
);

    reg [31:0] registers [0:31];
    
    initial begin
        registers[0] <= 32'd0;
        registers[29] <= 32'd51199;
        registers[28] <= 32'd6300;
    end

    always @(posedge en) begin
        read_data1 <= registers[read_reg1];
        read_data2 <= registers[read_reg2];
    end
    
    always @(posedge (en && reg_write)) begin
//        if (write_reg != 5'd0) begin
        registers[write_reg] <= write_data;
//        end
    end 
endmodule