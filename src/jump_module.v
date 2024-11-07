`timescale 1ns / 1ps

module JumpModule(
    input en,
    input jump,
    input [31:0] pc,
    input [25:0] addr,
    input [3:0] path_index,
    input [31:0] reg_addr,
    output reg [31:0] pc_out,
    output reg jump_done
);

    initial begin
        jump_done = 1'b0;
    end

    always @ (posedge en) begin
        if (jump & (path_index==4'd5 | path_index==4'd6)) begin
            pc_out <= {pc[31:28], addr, 2'b00};
        end
        else if(jump & path_index==4'd8) begin
            pc_out <= reg_addr;
        end
        else begin
            pc_out <= pc;
        end
        jump_done <= 1'b1;
    end

endmodule