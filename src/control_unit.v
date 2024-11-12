`timescale 1ns / 1ps

module ControlUnit(
    input fast_clk,
    input top_en,
    input infer,
    input [9:0] infer_addr,
//    output [31:0] infer_data,
    output reg ID,
    output reg IF,
    output reg REG,
    output reg EX,
    output reg MEM,
    output reg WB,
    output reg JU,
    output reg BR,
    output reg SK,
    output [6:0] LED_out,
    output [3:0] Anode_Activate
);
    
    // Variables
    reg [31:0] pc;
    reg [31:0] instr_reg;
    reg [3:0] state;
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

    RegisterFile regs(
    .en(reg_en),
    .reg_write(reg_write),
    .read_reg1(rs),
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
    assign infer_data = (infer)? mem_dout[15:0]: 16'd0;
    
    seven_seg_display seven_seg(
    .clock_100Mhz(fast_clk),
    .reset(0),
    .displayed_number(infer_data),
    .Anode_Activate(Anode_Activate),
    .LED_out(LED_out)
    );

    // STATES
    parameter IDLESRC = 4'b0000;
    parameter FETCH = 4'b0001;
    parameter RED1 = 4'b0010;
    parameter RED2 = 4'b0011;
    parameter DECODE = 4'b0100;
    parameter RED3  = 4'b0101;
    parameter REGFILE  = 4'b0110;
    parameter EXECUTE  = 4'b0111;
    parameter MEMORY  = 4'b1000;
    parameter RED4 = 4'b1001;
    parameter RED5 = 4'b1010;
    parameter REGWRITE  = 4'b1011;
    parameter JUMP = 4'b1100;
    parameter BRANCH = 4'b1101;
    parameter SINK = 4'b1110; 

    initial begin
        pc <= 32'd0;
        state <= IDLESRC;
        counter <= 4'd0;
        clk <= 1'd1;
    end
    
    always @ (posedge fast_clk) begin
        if (counter == 25'd1000000) begin
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
                    pc <= (Jump)? pc_out_j: (Branch)? pc_out_b: pc;
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
                branch_en <= 0;
                jump_en <= 0;
                alu_en <= 0;
            end
            RED1: begin
                state <= RED2;
            end
            RED2: begin 
                state <= DECODE;
            end
            DECODE: begin
                state <= RED3;
                instr_reg <= mem_dout;
                pc <= pc + 1;
                ID <= 1;
                IF <= 0;
                decoder_en <= 1;
            end
            RED3: begin
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
                mem_en <= 0;
                mem_ren <= 0;
                mem_wen <= 0;

            end
            REGFILE: begin
                    if ((path_index == 4'd1) | (path_index == 4'd2) | (path_index == 4'd3) | (path_index == 4'd4) | (path_index == 4'd7)) begin
                        state <= EXECUTE;
                    end
                    else begin //path_index == 4'd8
                        state <= JUMP;
                    end
                    decoder_en <= 0;
                    reg_en <= 1;
                    reg_write <= 0;
                    ID <= 0;
                    REG <= 1;
            end
            EXECUTE: begin
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
                    reg_en <= 0;
                    alu_en <= 1;
                    EX <= 1;
                    REG <= 0;
            end
            MEMORY: begin
                if (path_index == 4'd2) begin
                    state <= RED4;
                    mem_en <= 1;
                    mem_ren <= 1;
                    mem_wen <= 0;
                    mem_addr <= alu_result[15:0];
                end
                else begin // (path_index == 4'd3)
                    state <= FETCH;
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
            RED4: begin
                state <= RED5;
            end
            RED5: begin
                state <= REGWRITE;
            end
            REGWRITE: begin
                if (path_index == 4'd6) begin
                    state <= JUMP;
                end
                else begin // (path_index == 4'd0) | (path_index == 4'd1) | (path_index == 4'd2)
                    state <= FETCH;
                end
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
                    mem_en <= 1;
                    mem_ren <= 1;
                    mem_wen <= 0;
                    mem_addr <= infer_addr + 6300;
                end
                state <= SINK;
                ID <= 0;
                SK <= 1;
            end
        endcase    
    end

endmodule
