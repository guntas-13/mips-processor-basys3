`timescale 1ns / 1ps

module JumpModule(
    input clk,
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
        pc_out <= 32'd0;
    end
    
//    assign pc_o = (jump & (path_index==4'd5 | path_index==4'd6) & en)? {pc[31:26], addr} : (jump & path_index==4'd8 & en)? reg_addr: pc;

    always @ (posedge (clk & en)) begin
        if (en) begin
            if ((path_index==4'd5 | path_index==4'd6)) begin
                pc_out <= {pc[31:26], addr};
            end
            else if(path_index==4'd8) begin
                pc_out <= reg_addr;
            end
            else begin
                pc_out <= pc;
            end
            jump_done <= 1'b1;
        end
        else begin
            jump_done <= 1'b0;
        end
    end

endmodule