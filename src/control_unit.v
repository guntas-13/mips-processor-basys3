`timescale 1ns / 1ps

module ControlUnit(
    input fast_clk,
    input top_en,
    input infer,
    input [9:0] infer_addr,
//    output [31:0] infer_data,
    output reg IF,
    output reg ID,
    output reg REG,
    output reg EX,
    output reg MEM,
    output reg WB,
    output reg JU,
    output reg BR,
    output reg SK,
    output en_IF,
    output en_ID,
    output en_REG,
    output en_EX,
    output en_MEM,
    output en_WB,
    output [6:0] LED_out,
    output [3:0] Anode_Activate
);
    
    // Variables
    reg [31:0] pc;
    reg [31:0] instr_reg;
    reg [4:0] state;
    wire [3:0] path_index;
    reg [24:0] counter;
    reg clk;

    // Regfile
    reg reg_en;
    reg reg_write;
    wire [4:0] rs;
    wire [4:0] rt;
    wire [4:0] write_reg;
    wire [31:0] write_data;
    wire [31:0] read_data1;
    wire [31:0] read_data2;

    wire [4:0] read_reg1;
    assign read_reg1 = (infer)? infer_addr[4:0]: rs;
    
    RegisterFile regs(
    .en(reg_en),
    .reg_write(reg_write),
    .read_reg1(read_reg1),
    .read_reg2(rt),
    .write_reg(write_reg),
    .write_data(write_data),
    .read_data1(read_data1),
    .read_data2(read_data2)
    );

    // ALU
    reg alu_en;
    wire [3:0] ALU_Control; // this is the input [3:0] alu_control in the alu.v
    wire [31:0] alu_srcA;
    wire [31:0] alu_srcB;
    wire [31:0] alu_result;
    wire [31:0] hi;
    wire [31:0] lo;
    wire overflow;
    wire alu_zero;
    

    ALU alu(
    .en(alu_en),
    .alu_control(ALU_Control),
    .alu_srcA(alu_srcA),          // this corresponds to $rs or shamt
    .alu_srcB(alu_srcB),          // this corresponds to $rt
    .alu_result(alu_result),   // this corresponds to $rd
    .hi(hi),
    .lo(lo),
    .overflow(overflow),
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
    wire RegDst; // will go as select line in the MUX for instr[20:16] (rt) or instr[15:11] (rd)
    wire Jump;
    wire Branch;
    wire [1:0] MemtoReg; // this is the select line in the MUX for the data to be written to the register file from the read data from the memory or the ALU output
    wire ALUSrc; // this is the select line in the MUX for the second operand of the ALU (either the immediate value or the value from the register file)
    wire [31:0] imm_extended; // this is the immediate value extended to 32 bits
    wire [4:0] rd;
    wire [4:0] shamt;
    wire [25:0] jump_address;
    wire select_shamt;

    decoder_control decoder(
    .en(decoder_en),
    .instr(instr_reg),
    .RegDst(RegDst),
    .Jump(Jump),
    .Branch(Branch),
    .MemtoReg(MemtoReg),
    .ALU_Control(ALU_Control),
    .ALUSrc(ALUSrc),
    .imm_extended(imm_extended),
    .rs(rs),
    .rt(rt),
    .rd(rd),
    .shamt(shamt),
    .jump_address(jump_address),
    .path_index(path_index),
    .select_shamt(select_shamt)
    );

    //Branch
    reg branch_en;
    wire [31:0] pc_out_b;

    BranchModule branch(
    .en(branch_en),
    .alu_zero(alu_zero),
    .imm(imm_extended),
    .pc(pc),
    .pc_out(pc_out_b)
    );

    //Jump 
    reg jump_en;
    wire [31:0] pc_out_j;
    
    JumpModule jump(
    .en(jump_en),
    .pc(pc),
    .addr(jump_address),
    .path_index(path_index),
    .reg_addr(read_data1),
    .pc_out(pc_out_j)
    );

    // MUX
    ALU_SRC_MUX alu_srcA_mux(
    .read_data1(read_data1),
    .shamt(shamt),
    .select_shamt(select_shamt),
    .alu_srcA(alu_srcA)
    );

    MUX2x1 alu_srcB_mux(
    .input0(read_data2),
    .input1(imm_extended),
    .SEL(ALUSrc),
    .mux_out(alu_srcB)
    );
    
    MUX4x2_RegFile write_reg_mux(
    .rt(rt),
    .rd(rd),
    .fix_ra(5'b11111),
    .SEL0_RegDst(RegDst),
    .SEL1_Jump(Jump),
    .mux_out(write_reg)
    );

    MUX8x3_RegWrite write_reg_data_mux(
    .alu_result(alu_result),
    .dout(mem_dout),
    .hi(hi),
    .lo(lo),
    .PC_updated(pc),   // For jal instruction need PC + 4 to be written to register file
    .SEL0_MemtoReg0(MemtoReg[0]),
    .SEL1_MemtoReg1(MemtoReg[1]),
    .SEL2_Jump(Jump),            // For jal instruction need PC + 4 to be written to register file
    .mux_out(write_data)
    );
    
    wire [15:0] infer_data;
    assign infer_data = (infer)? read_data1[15:0]: 16'd0;
    
    seven_seg_display seven_seg(
    .clock_100Mhz(fast_clk),
    .reset(0),
    .displayed_number(infer_data),
    .Anode_Activate(Anode_Activate),
    .LED_out(LED_out)
    );

    // STATES
    parameter IDLESRC = 5'b00000;
    parameter FETCH = 5'b00001;
    parameter RED1 = 5'b00010;
    parameter RED2 = 5'b00011;
    parameter RED3  = 5'b00100;
    parameter DECODE = 5'b00101;
    parameter RED4 = 5'b00110;
    parameter REGFILE  = 5'b00111;
    parameter RED9 = 5'b10010;
    parameter EXECUTE  = 5'b01000;
    parameter RED10 = 5'b10011;
    parameter MEMORY  = 5'b01001;
    parameter RED5 = 5'b01010;
    parameter RED6 = 5'b01011;
    parameter RED7 = 5'b01100;
    parameter REGWRITE = 5'b01101;
    parameter RED8 = 5'b01110;
    parameter JUMP = 5'b01111;
    parameter BRANCH = 5'b10000;
    parameter SINK = 5'b10001; 

    reg [4:0] reg_cnt;
    initial begin
        pc <= 32'd0;
        state <= IDLESRC;
        counter <= 4'd0;
        clk <= 1'd1;
        reg_cnt <= 5'd0;
    end
    
    assign en_IF = mem_en;
    assign en_ID = decoder_en;
    assign en_REG = reg_en;
    assign en_EX = alu_en;
    assign en_MEM = mem_en;
    assign en_WB = reg_en;   
    
    always @ (posedge fast_clk) begin
        if (counter == 25'd15000000) begin //15000000
            counter <= 0;
            clk <= ~clk;
        end
        else begin
            counter <= counter + 1;
        end        
    end
    always @ (posedge (clk & top_en))
    begin
        case(state)
            IDLESRC: begin
                if (top_en) state <= FETCH;
                else state <= IDLESRC;
                IF <= 0;
                ID <= 0;
                REG <= 0;
                EX <= 0;
                MEM <= 0;
                WB <= 0;
                JU <= 0;
                BR <= 0;
                SK <= 0;
            end
            FETCH: begin
                if (pc == 32'd0) begin
                    mem_addr <= pc[15:0]; 
                end
                else begin
                    mem_addr <= (Jump)? pc_out_j[15:0]: (Branch)? pc_out_b[15:0]: pc[15:0];
                end
                state <= RED1;
                mem_en <= 1;
                mem_ren <= 1;
                mem_wen <= 0;
                IF <= 1;
                ID <= 0;
                REG <= 0;
                EX <= 0;
                MEM <= 0;
                WB <= 0;
                JU <= 0;
                BR <= 0;
                SK <= 0;
                reg_en <= 0;
                reg_write <= 0;
                alu_en <= 0;
            end
            RED1: begin
                state <= RED2;
                if (pc == 32'd0) begin
                    pc <= pc; 
                end
                else begin
                    pc <= (Jump)? pc_out_j: (Branch)? pc_out_b: pc;
                end
            end
            RED2: begin 
                state <= RED3;
                jump_en <= 0;
                branch_en <= 0;
            end
            RED3: begin 
                state <= DECODE;
                instr_reg <= mem_dout;
            end
            DECODE: begin
                state <= RED4;
                pc <= pc + 1;
                ID <= 1;
                IF <= 0;
                decoder_en <= 1;
                mem_en <= 0;
                mem_ren <= 0;
                mem_wen <= 0;
            end
            RED4: begin
                if ((path_index == 4'd0)|(path_index == 4'd6))begin
                    state <= REGWRITE;
                end
                else if (path_index == 4'd5) begin
                    state <= JUMP;
                end
                else if (path_index == 4'd9) begin
                    state <= SINK;
                end 
                else state <= REGFILE; // many instructions

            end
            REGFILE: begin
                    decoder_en <= 0;
                    reg_en <= 1;
                    reg_write <= 0;
                    ID <= 0;
                    REG <= 1;
                    state <= RED9;
            end
            RED9: begin
                if ((path_index == 4'd1) | (path_index == 4'd2) | (path_index == 4'd3) | (path_index == 4'd4) | (path_index == 4'd7)) begin
                        state <= EXECUTE;
                    end
                    else begin //path_index == 4'd8
                        state <= JUMP;
                    end
            end
            EXECUTE: begin
                    reg_en <= 0;
                    alu_en <= 1;
                    EX <= 1;
                    REG <= 0;
                    state <= RED10;
            end
            RED10: begin
                if (path_index == 4'd1) begin
                        state <= REGWRITE;
                    end
                    else if ((path_index == 4'd2) | (path_index == 4'd3)) begin
                        state <= MEMORY;
                    end
                    else if (path_index == 4'd7) begin
                        state <= FETCH;
                    end
                    else begin // (path_index == 4'd4)
                        state <= BRANCH;
                    end
            end
            MEMORY: begin
                if (path_index == 4'd2) begin
                    state <= RED5;
                    mem_en <= 1;
                    mem_ren <= 1;
                    mem_wen <= 0;
                    mem_addr <= alu_result[15:0];
                end
                else begin // (path_index == 4'd3)
                    state <= RED5;
                    mem_en <= 1;
                    mem_wen <= 1;
                    mem_ren <= 0;
                    mem_addr <= alu_result[15:0];
                    mem_din <= read_data2;
                end
                
                alu_en <= 0;
                MEM <= 1;
                EX <= 0;
            end
            RED5: begin
                if (path_index == 4'd2) begin
                    state <= RED6;
                end
                else begin // (path_index == 4'd3)
                    state <= FETCH;
                end
            end
            RED6: begin
                state <= RED7;
            end
            RED7: begin
                state <= REGWRITE;
            end
            REGWRITE: begin
                state <= RED8;
                WB <= 1;
                ID <= 0;
                EX <= 0;
                MEM <= 0;
                mem_en <= 0;
                mem_ren <= 0;
                mem_wen <= 0;
                decoder_en <= 0;
                alu_en <= 0;
                reg_en <= 1;
                reg_write <= 1;
            end
            RED8: begin
                if (path_index == 4'd6) begin
                    state <= JUMP;
                end
                else begin // (path_index == 4'd0) | (path_index == 4'd1) | (path_index == 4'd2)
                    state <= FETCH;
                end
            end
            JUMP: begin
                state <= FETCH;
                JU <= 1;
                WB <= 0;
                ID <= 0;
                REG <= 0;
                mem_en <= 0;
                mem_ren <= 0;
                mem_wen <= 0;
                decoder_en <= 0;
                reg_en <= 0;
                reg_write <= 0;
                jump_en <= 1;
            end
            BRANCH: begin
                state <= FETCH;
                alu_en <= 0;
                branch_en <= 1;
                BR <= 1;
                EX <= 0;
            end
            SINK: begin
                if (infer) begin
                    if (reg_cnt == 5'd1) begin
                        reg_en <= ~reg_en;
                        reg_cnt <= 0;
                    end
                    else begin
                        reg_cnt <= reg_cnt + 1;
                    end
                end
                state <= SINK;
                ID <= 0;
                SK <= 1;
                decoder_en <= 0;
            end
        endcase    
    end

endmodule
