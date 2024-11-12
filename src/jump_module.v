`timescale 1ns / 1ps

module JumpModule(
    input en,
    input [31:0] pc,
    input [25:0] addr,
    input [3:0] path_index,
    input [31:0] reg_addr,
    output reg [31:0] pc_out
);
    
    initial begin
        pc_out <= 32'd0;
    end
    
    always @ (posedge en) begin
        if ((path_index==4'd5 | path_index==4'd6)) begin
            pc_out <= {pc[31:26], addr};
        end
        else if(path_index==4'd8) begin
            pc_out <= reg_addr;
        end
        else begin
            pc_out <= pc;
        end
    end

endmodule