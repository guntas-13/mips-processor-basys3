`timescale 1ns / 1ps

module ControlUnit(
input clk,
output ID,
output IF,
output EX,
output MEM,
output WB,
output JU,
output BR
);

// Variables
reg [31:0] registers [0:31];
reg [31:0] pc;
wire [31:0] instr_reg;

// ALU
reg alu_en;
reg [3:0] alu_control;
reg 
[31:0] alu_srcA;
reg [31:0] alu_srcB;
reg [31:0] alu_result;
reg [31:0] hi;
reg [31:0] lo;
reg overflow;
reg alu_done;
wire alu_zero;

// MEM
reg mem_en;
reg mem_wen;
reg [15:0] mem_addr;
reg [31:0] mem_din;
wire [31:0] mem_dout;

//Decoder
reg decoder_en;
reg RegDst; // will go as select line in the MUX for instr[20:16] (rt) or instr[15:11] (rd)
reg Jump;
reg Branch;
reg MemRead; // this is the input ren in MemReadWrite.v
reg MemWrite; // this is the input wen in MemReadWrite.v
reg MemtoReg; // this is the select line in the MUX for the data to be written to the register file from the read data from the memory or the ALU output
reg [3:0] ALU_Control; // this is the input [3:0] alu_control in the alu.v
reg ALUSrc; // this is the select line in the MUX for the second operand of the ALU (either the immediate value or the value from the register file)
reg RegWrite; // this is the input reg_write in the register_file.v
reg [31:0] imm_extended; // this is the immediate value extended to 32 bits
reg [4:0] rs;
reg [4:0] rt;
reg [4:0] rd;
reg [4:0] shamt;
reg [25:0] jump_address;

//Branch
reg branch_en;
wire [31:0] pc_out;

//Jump


initial begin
    pc <= 32'd0;
end

always @ (posedge clk)
begin
    
end




endmodule
