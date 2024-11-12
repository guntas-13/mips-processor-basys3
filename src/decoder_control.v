`timescale 1ns/1ps

module decoder_control(
    input en,
    input [31:0] instr,
    output reg RegDst, // will go as select line in the MUX for instr[20:16] (rt) or instr[15:11] (rd)
    output reg Jump,
    output reg Branch,
    output reg [1:0] MemtoReg, //These are the 2 select lines to choose from 4 options in the MUX for the write data to the register file -> 00: alu_result (in alu.v), 01: dout (in MemReadWrite.v), 10: hi, 11: lo (both in alu.v)
    output reg [3:0] ALU_Control, // this is the input [3:0] alu_control in the alu.v
    output reg ALUSrc, // this is the select line in the MUX for the second operand of the ALU (either the immediate value or the value from the register file)
    output reg [31:0] imm_extended, // this is the immediate value extended to 32 bits
    output [4:0] rs,
    output [4:0] rt,
    output [4:0] rd,
    output [4:0] shamt,
    output [25:0] jump_address,
    output reg [3:0] path_index,
    output reg select_shamt
);

    wire [5:0] opcode;
    wire [5:0] funct;
    wire [15:0] imm;
    assign opcode = instr[31:26];
    assign funct = instr[5:0];
    assign imm = instr[15:0];
    assign rs = instr[25:21];
    assign rt = instr[20:16];
    assign rd = instr[15:11];
    assign shamt = instr[10:6];
    assign jump_address = instr[25:0];

    always @(posedge  en) begin
        if(imm[15] == 1'b1) begin
            imm_extended <= {16'b1111111111111111, imm};
        end
        else begin
            imm_extended <= {16'b0000000000000000, imm};
        end

        case(opcode)
            6'b000000: begin // R-type
                RegDst <= 1'b1;
                Jump <= 1'b0;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                select_shamt <= 1'b0;

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
                    path_index <= 4'b0001;
                end
                else if (funct == 6'b100010) begin // sub
                    ALU_Control <= 4'b0001;
                    path_index <= 4'b0001;
                end
                else if (funct == 6'b100100) begin // and
                    ALU_Control <= 4'b0010;
                    path_index <= 4'b0001;
                end
                else if (funct == 6'b100101) begin // or
                    ALU_Control <= 4'b0011;
                    path_index <= 4'b0001;
                end
                else if (funct == 6'b101010) begin // slt
                    ALU_Control <= 4'b0101;
                    path_index <= 4'b0001;
                end
                else if (funct == 6'b100111) begin // nor
                    ALU_Control <= 4'b0100;
                    path_index <= 4'b0001;
                end
                else if (funct == 6'b000000) begin // sll
                    ALU_Control <= 4'b0110;
                    select_shamt <= 1'b1;
                    path_index <= 4'b0001;
                end
                else if (funct == 6'b000010) begin // srl
                    ALU_Control <= 4'b0111;
                    select_shamt <= 1'b1;
                    path_index <= 4'b0001;
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
                
                ALUSrc <= 1'b0;

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
                MemtoReg <= 2'b01;
                ALU_Control <= 4'b0000;
                ALUSrc <= 1'b1;
                path_index <= 4'b0010;
                select_shamt <= 1'b0;
                
            end
            6'b101011: begin // sw
                RegDst <= 1'b0;
                Jump <= 1'b0;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0000;
                ALUSrc <= 1'b1;
                path_index <= 4'b0011;
                select_shamt <= 1'b0;
                
            end
            6'b000100: begin // beq
                RegDst <= 1'b0;
                Jump <= 1'b0;
                Branch <= 1'b1;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0001; // subtract -> if alu_zero is 1, then branch i.e. alu_result of subtract is 0
                ALUSrc <= 1'b0;
                path_index <= 4'b0100;
                select_shamt <= 1'b0;
                
            end
            6'b001000: begin // addi
                RegDst <= 1'b0;
                Jump <= 1'b0;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0000;
                ALUSrc <= 1'b1;
                path_index <= 4'b0001; // This is handled in R Type's path itself
                select_shamt <= 1'b0;
                
            end
            6'b001010: begin // slti
                RegDst <= 1'b0;
                Jump <= 1'b0;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0101;
                ALUSrc <= 1'b1;
                path_index <= 4'b0001; // This is handled in R Type's path itself
                select_shamt <= 1'b0;
                
            end
            6'b001100: begin // andi
                RegDst <= 1'b0;
                Jump <= 1'b0;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0010;
                ALUSrc <= 1'b1;
                path_index <= 4'b0001; // This is handled in R Type's path itself
                select_shamt <= 1'b0;
                
            end
            6'b001101: begin // ori
                RegDst <= 1'b0;
                Jump <= 1'b0;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0011;
                ALUSrc <= 1'b1;
                path_index <= 4'b0001; // This is handled in R Type's path itself
                select_shamt <= 1'b0;             
            end
            // J-Type
            6'b000010: begin // j
                RegDst <= 1'b0;
                Jump <= 1'b1;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0000; // dummy
                ALUSrc <= 1'b0;
                path_index <= 4'b0101;
                select_shamt <= 1'b0;
            end
            6'b000011: begin // jal
                RegDst <= 1'b0;
                Jump <= 1'b1;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0000; // dummy
                ALUSrc <= 1'b0;
                path_index <= 4'b0110;
                select_shamt <= 1'b0;
                
            end
            6'b111111: begin // exit
                RegDst <= 1'b0;
                Jump <= 1'b0;
                Branch <= 1'b0;
                MemtoReg <= 2'b00;
                ALU_Control <= 4'b0000; // dummy
                ALUSrc <= 1'b0;
                path_index <= 4'b1001;
                select_shamt <= 1'b0;
            end
        endcase
    end

endmodule