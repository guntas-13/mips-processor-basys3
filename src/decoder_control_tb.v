`timescale 1ns / 1ps

module decoder_control_tb();

reg clk;
reg decoder_en;
reg [31:0] instr_reg;

wire RegDst; // will go as select line in the MUX for instr[20:16] (rt) or instr[15:11] (rd)
wire Jump;
wire Branch;
wire MemRead; // this is the input ren in MemReadWrite.v
wire MemWrite; // this is the input wen in MemReadWrite.v
wire [1:0] MemtoReg; // this is the select line in the MUX for the data to be written to the register file from the read data from the memory or the ALU output
wire [3:0] ALU_Control; // this is the input [3:0] alu_control in the alu.v
wire ALUSrc; // this is the select line in the MUX for the second operand of the ALU (either the immediate value or the value from the register file)
wire RegWrite; // this is the input reg_write in the register_file.v
wire [31:0] imm_extended; // this is the immediate value extended to 32 bits
wire [4:0] rs;
wire [4:0] rt;
wire [4:0] rd;
wire [4:0] shamt;
wire [25:0] jump_address;
wire decoder_done;
wire select_shamt;

decoder_control uut(
.clk(clk),
.en(decoder_en),
.instr(instr_reg),
.RegDst(RegDst),
.Jump(Jump),
.Branch(Branch),
.MemRead(MemRead),
.MemtoReg(MemtoReg),
.ALU_Control(ALU_Control),
.MemWrite(MemWrite),
.ALUSrc(ALUSrc),
.RegWrite(RegWrite),
.imm_extended(imm_extended),
.rs(rs),
.rt(rt),
.rd(rd),
.shamt(shamt),
.jump_address(jump_address),
.path_index(path_index),
.decoder_done(decoder_done),
.select_shamt(select_shamt)
);

initial begin
    clk = 1;
    forever #0.5 clk = ~clk;
end

initial begin
    decoder_en <= 0;
    instr_reg <= 32'b10001011100010000000000000000000;
    #5;
    decoder_en <= 1;
    #20;
    $finish();
    
end

endmodule
