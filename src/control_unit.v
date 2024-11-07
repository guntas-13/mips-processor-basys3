`timescale 1ns / 1ps

module ControlUnit(
    input clk,
    input top_en,
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
    reg [3:0] path_index;
    reg [31:0] pc_out;
    reg [3:0] state;

    // Regfile
    reg reg_en;
    reg reg_write;
    reg [4:0] read_reg1;
    reg [4:0] read_reg2;
    reg [4:0] write_reg;
    reg [31:0] write_data;
    reg [31:0] read_data1;
    reg [31:0] read_data2;
    reg register_done;

    RegisterFile regs(
    .clk(clk),
    .en(reg_en),
    .reg_write(reg_write),
    .read_reg1(read_reg1),
    .read_reg2(read_reg2),
    .write_reg(write_reg),
    .write_data(write_data),
    .read_data1(read_data1),
    .read_data2(read_data2),
    .register_done(register_done)
    );

    // ALU
    reg alu_en;
    reg [3:0] alu_control;
    reg [31:0] alu_srcA;
    reg [31:0] alu_srcB;
    reg [31:0] alu_result;
    reg [31:0] hi;
    reg [31:0] lo;
    reg overflow;
    reg alu_done;
    wire alu_zero;

    ALU alu(
    .clk(clk),
    .en(alu_en),
    .alu_control(alu_control),
    .alu_srcA(alu_srcA),          // this corresponds to $rs or shamt
    .alu_srcB(alu_srcB),          // this corresponds to $rt
    .alu_result(alu_result),   // this corresponds to $rd
    .hi(hi),
    .lo(lo),
    .overflow(overflow),
    .alu_done(alu_done),
    .alu_zero(alu_zero)
    );

    // MEM
    reg mem_en;
    reg mem_ren;
    reg mem_wen;
    reg [15:0] mem_addr;
    reg [31:0] mem_din;
    wire [31:0] mem_dout;

    MemReadWrite mem(
    .clk(clk),
    .en(mem_en),
    .ren(mem_ren),
    .wen(mem_wen),
    .addr(mem_addr),
    .din(mem_din),
    .dout(mem_dout)
    );

    //Decoder
    reg decoder_en;
    reg RegDst; // will go as select line in the MUX for instr[20:16] (rt) or instr[15:11] (rd)
    reg Jump;
    reg Branch;
    reg MemRead; // this is the input ren in MemReadWrite.v
    reg MemWrite; // this is the input wen in MemReadWrite.v
    reg [1:0] MemtoReg; // this is the select line in the MUX for the data to be written to the register file from the read data from the memory or the ALU output
    reg [3:0] ALU_Control; // this is the input [3:0] alu_control in the alu.v
    reg ALUSrc; // this is the select line in the MUX for the second operand of the ALU (either the immediate value or the value from the register file)
    reg RegWrite; // this is the input reg_write in the register_file.v
    reg [31:0] imm_extended; // this is the immediate value extended to 32 bits
    reg [4:0] rs;
    reg [4:0] rt;
    reg [4:0] rd;
    reg [4:0] shamt;
    reg [25:0] jump_address;
    reg decoder_done;

    decoder_control decoder(
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
    .path_index(path_index)
    .decoder_done(decoder_done)
    );

    //Branch
    reg branch_en;
    reg branch_done;

    BranchModule branch(
    .en(branch_en),
    .branch(Branch),
    .alu_zero(alu_zero),
    .imm(imm_extended),
    .pc(pc),
    .pc_out(pc_out),
    .branch_done(branch_done)
    );

    //Jump 
    reg jump_en;
    reg jump_done;

    JumpModule jump(
    .en(jump_en),
    .jump(Jump),
    .pc(pc),
    .addr(jump_address),
    .path_index(path_index),
    .reg_addr(read_data1),
    .pc_out(pc_out),
    .jump_done(jump_done)
    );

    // STATES
    parameter s0  = 4'b0000; // Idle State
    parameter s1  = 4'b0001; // IF
    parameter s2  = 4'b0010; // Redundant State
    parameter s3   = 4'b0011; // Redundant State
    parameter s4  = 4'b0100; // ID
    parameter s5  = 4'b0101; // REG
    parameter s6  = 4'b0110;
    parameter s7  = 4'b0111;
    parameter s8 = 4'b1000;
    parameter s9  = 4'b1001;
    parameter s10  = 4'b1010;


    initial begin
        pc <= 32'd0;
        pc_out <= 32'd0;
        state <= s0;
    end

    
    always @ (posedge clk)
    begin
        case(state) begin
            s0: begin // Idle State
                if (top_en) state <= s1;
                else state <= s0;
            end
            s1: begin // IF
                if(mem_done) begin
                    state <= s2;
                    mem_done <= 0;
                    mem_en <= 0;
                end
                else begin
                    state <= s1;
                    mem_en <= 1;
                    IF <= 1;
                end
            end
            s2: begin // Redundant State
                state <= s3;
            end
            s3: begin // Redundant State
                state <= s4;
            end
            s4: begin // ID
                if(decoder_done) begin
                    if (path_index == 4'd0) begin
                        // state <= write_back;
                    end
                    else if (path_index ==4'd5) begin
                        // state <= jump;
                    else state <= s5; // REG
                    decoder_done <= 0;
                    decoder_en <= 0;
                end
                else begin
                    state <= s4;
                    decoder_en <= 1;
                    ID <= 1;
                    IF <= 0;
                end
            end
        end
        
    end




endmodule