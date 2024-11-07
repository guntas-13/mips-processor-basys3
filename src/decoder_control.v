`timescale 1ns/1ps

module decoder_control(
    input clk,
    input en,
    input [31:0] instr,
    output reg RegDst, // will go as select line in the MUX for instr[20:16] (rt) or instr[15:11] (rd)
    output reg Jump,
    output reg Branch,
    output reg MemRead, // this is the input ren in MemReadWrite.v
    output reg [1:0] MemtoReg, //These are the 2 select lines to choose from 4 options in the MUX for the write data to the register file -> 00: alu_result (in alu.v), 01: dout (in MemReadWrite.v), 10: hi, 11: lo (both in alu.v)
    output reg [3:0] ALU_Control, // this is the input [3:0] alu_control in the alu.v
    output reg MemWrite, // this is the input wen in MemReadWrite.v
    output reg ALUSrc, // this is the select line in the MUX for the second operand of the ALU (either the immediate value or the value from the register file)
    output reg RegWrite // this is the input reg_write in the register_file.v
    output reg [31:0] imm_extended, // this is the immediate value extended to 32 bits
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] rd,
    output reg [4:0] shamt,
    output reg [25:0] jump_address,
    output reg [3:0] path_index;
);

    reg [5:0] opcode;
    reg [5:0] funct;
    reg [15:0] imm;

    always @(posedge clk) begin
        if (en) begin
            opcode <= instr[31:26];
            funct <= instr[5:0];
            rs <= instr[25:21];
            rt <= instr[20:16];
            rd <= instr[15:11];
            shamt <= instr[10:6];
            imm <= instr[15:0];
            jump_address <= instr[25:0];
            // imm_extended <= {imm[15], imm};
            if(imm[15] == 1'b1) begin
                imm_extended <= {16'b1111111111111111, imm};
            end
            else begin
                imm_extended <= {16'b0000000000000000, imm};
            end
        end
    end

    always @(posedge clk) begin
        if (en) begin
            case(opcode)
                6'b000000: begin // R-type
                    RegDst <= 1'b1;
                    Jump <= 1'b0;
                    Branch <= 1'b0;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    path_index <= 4'b0001;

                    /*
                    we have 10 operations defined in the alu.v
                    ALU_Control = 0000: ADD -> funct = 100000
                    ALU_Control = 0001: SUB -> funct = 100010
                    ALU_Control = 0010: AND -> funct = 100100
                    ALU_Control = 0011: OR -> funct = 100101
                    ALU_Control = 0100: NOR -> funct = 100111
                    ALU_Control = 0101: SLT -> funct = 101010
                    ALU_Control = 0110: SLL -> funct = 000000
                    ALU_Control = 0111: SRL -> funct = 000010
                    ALU_Control = 1000: MULT -> funct = 011000
                    ALU_Control = 1001: DIV -> funct = 011010
                    */
                    
                    if (funct == 6'b100000) begin // add
                        ALU_Control <= 4'b0000;
                    end
                    else if (funct == 6'b100010) begin // sub
                        ALU_Control <= 4'b0001;
                    end
                    else if (funct == 6'b100100) begin // and
                        ALU_Control <= 4'b0010;
                    end
                    else if (funct == 6'b100101) begin // or
                        ALU_Control <= 4'b0011;
                    end
                    else if (funct == 6'b101010) begin // slt
                        ALU_Control <= 4'b0101;
                    end
                    else if (funct == 6'b100111) begin // nor
                        ALU_Control <= 4'b0100;
                    end
                    else if (funct == 6'b000000) begin // sll
                        ALU_Control <= 4'b0110;
                    end
                    else if (funct == 6'b000010) begin // srl
                        ALU_Control <= 4'b0111;
                    end
                    else if (funct == 6'b011000) begin // mult
                        ALU_Control <= 4'b1000;
                        path_index <= 4'b0111;
                    end
                    else if (funct == 6'b011010) begin // div
                        ALU_Control <= 4'b1001;
                        path_index <= 4'b0111;
                    end
                    else begin
                        ALU_Control <= 4'b0000;
                    end

                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b0;
                    RegWrite <= 1'b1;

                    // for mflo and mfhi
                    if (funct == 6'b010000) begin // mflo
                        MemtoReg <= 2'b11;
                        path_index <= 4'b0000;
                    end
                    else if (funct == 6'b010010) begin // mfhi
                        MemtoReg <= 2'b10;
                        path_index <= 4'b0000;
                    end
                    else begin
                        MemtoReg <= 2'b00;
                    end

                    // ONLY jr is left -> We'll see
                    if (funct == 6'b001000) begin // jr $rs
                        Jump <= 1'b1;
                        path_index <= 4'b1000;
                    end
                    else begin
                        Jump <= 1'b0;
                    end

                end
                6'b100010: begin // lw
                    RegDst <= 1'b0;
                    Jump <= 1'b0;
                    Branch <= 1'b0;
                    MemRead <= 1'b1;
                    MemtoReg <= 2'b01;
                    ALU_Control <= 4'b0000;
                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b1;
                    RegWrite <= 1'b1;
                    path_index <= 4'b0010;
                end
                6'b101011: begin // sw
                    RegDst <= 1'b0;
                    Jump <= 1'b0;
                    Branch <= 1'b0;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    ALU_Control <= 4'b0000;
                    MemWrite <= 1'b1;
                    ALUSrc <= 1'b1;
                    RegWrite <= 1'b0; // in the MemReadWrite.v, we have to send input [31:0] din as output reg [31:0] read_data2 in register_file.v
                    path_index <= 4'b0011;
                end
                6'b000100: begin // beq
                    RegDst <= 1'b0;
                    Jump <= 1'b0;
                    Branch <= 1'b1;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    ALU_Control <= 4'b0001; // subtract -> if alu_zero is 1, then branch i.e. alu_result of subtract is 0
                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b0;
                    RegWrite <= 1'b0;
                    path_index <= 4'b0100;
                end
                6'b001000: begin // addi
                    RegDst <= 1'b0;
                    Jump <= 1'b0;
                    Branch <= 1'b0;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    ALU_Control <= 4'b0000;
                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b1;
                    RegWrite <= 1'b1;
                    path_index <= 4'b0001; // This is handled in R Type's path itself
                end
                6'b001010: begin // slti
                    RegDst <= 1'b0;
                    Jump <= 1'b0;
                    Branch <= 1'b0;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    ALU_Control <= 4'b0101;
                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b1;
                    RegWrite <= 1'b1;
                    path_index <= 4'b0001; // This is handled in R Type's path itself
                end
                6'b001100: begin // andi
                    RegDst <= 1'b0;
                    Jump <= 1'b0;
                    Branch <= 1'b0;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    ALU_Control <= 4'b0010;
                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b1;
                    RegWrite <= 1'b1;
                    path_index <= 4'b0001; // This is handled in R Type's path itself
                end
                6'b001101: begin // ori
                    RegDst <= 1'b0;
                    Jump <= 1'b0;
                    Branch <= 1'b0;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    ALU_Control <= 4'b0011;
                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b1;
                    RegWrite <= 1'b1;
                    path_index <= 4'b0001; // This is handled in R Type's path itself
                end
                // J-Type
                6'b000010: begin // j
                    RegDst <= 1'b0;
                    Jump <= 1'b1;
                    Branch <= 1'b0;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    ALU_Control <= 4'b0000; // dummy
                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b0;
                    RegWrite <= 1'b0;
                    path_index <= 4'b0101;
                end
                6'b000011: begin // jal
                    RegDst <= 1'b0;
                    Jump <= 1'b1;
                    Branch <= 1'b0;
                    MemRead <= 1'b0;
                    MemtoReg <= 2'b00;
                    ALU_Control <= 4'b0000; // dummy
                    MemWrite <= 1'b0;
                    ALUSrc <= 1'b0;
                    RegWrite <= 1'b1;
                    path_index <= 4'b0110;
                end

            endcase
        end
    end

endmodule