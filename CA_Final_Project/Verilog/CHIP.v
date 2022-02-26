// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    wire   [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    wire Branch, MemRead, MemToReg, MemWrite, AluSrc, Jalr, Jal, Aupic;
    wire [2:0] type;
    wire [3:0] mode, AluOp;
    wire zero;
    wire [31:0] alu_o_basic, alu_o_muldiv, aluO;
    wire [31:0] in_A, in_B, imm;
    wire valid, ready;

    //type
    parameter R = 3'b000;
    parameter I = 3'b001;
    parameter S = 3'b010;
    parameter B = 3'b011;
    parameter JAL = 3'b100;
    parameter JALR = 3'b101;
    parameter AUPIC = 3'b110;
    parameter LW = 3'b111;
    //mode
    parameter MUL = 4'b0000;
    parameter DIV = 4'b0001;
    parameter AND = 4'b0010;
    parameter OR = 4'b0011;
    parameter ADD = 4'b0100;
    parameter SUB = 4'b0101;
    parameter LT = 4'b0110;
    parameter SL = 4'b0111;
    parameter SR = 4'b1000;

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    
    // Todo: any combinational/sequential circuit

    assign valid = (mode == MUL || mode == DIV) ? 1'b1 : 1'b0;
    assign in_A = (type == AUPIC) ? PC : rs1_data;
    assign in_B = (AluSrc) ?  imm : rs2_data;
    assign aluO = (mode == MUL || mode == DIV) ? alu_o_muldiv : alu_o_basic;
    assign rd_data = (type == JAL || type == JALR) ? (PC + 32'd4) : MemToReg ? mem_rdata_D : aluO;

    assign mem_wen_D = MemWrite;
    assign mem_addr_D = alu_o_basic;
    assign mem_wdata_D = rs2_data;
    assign mem_addr_I = PC;

    PC pc0(
        .clk(clk), 
        .rst_n(rst_n), 
        .type(type), 
        .mode(AluOp), 
        .imm(imm), 
        .zero(zero), 
        .rs1_data(rs1_data),  
        .PC_o(PC)
    );

    BasicALU basicALU0(
        .clk(clk), 
        .rst_n(rst_n), 
        .mode(AluOp), 
        .in_A(in_A), 
        .in_B(in_B), 
        .out(alu_o_basic), 
        .zero(zero)
    );

    MulDiv mulDiv0(
        .clk(clk), 
        .rst_n(rst_n), 
        .valid(valid), 
        .ready(ready), 
        .mode(AluOp), 
        .in_A(rs1_data), 
        .in_B(rs2_data), 
        .out(alu_o_muldiv)
    );

    Control ctrl0(
        .clk(clk), 
        .rst_n(rst_n),
        .mem_rdata_I(mem_rdata_I),
        .Branch_o(Branch), 
        .MemRead_o(MemRead), 
        .MemToReg_o(MemToReg), 
        .MemWrite_o(MemWrite), 
        .AluSrc_o(AluSrc), 
        .RegWrite_o(regWrite), 
        .Jalr_o(Jalr), 
        .Jal_o(Jal), 
        .Aupic_o(Aupic),
        .rs1(rs1), 
        .rs2(rs2), 
        .rd(rd),
        .imm_o(imm),
        .AluOp_o(AluOp),
        .type_o(type),
        .mode_o(mode)
    );


    
endmodule

module PC(clk, rst_n, type, mode, imm, zero, rs1_data, PC_o);
    input clk, rst_n;
    input [2:0] type;
    input [3:0] mode;
    input [31:0] imm, rs1_data;
    input zero;
    output [31:0] PC_o;

    reg [31:0] PC;
    reg [31:0] PC_nxt;
    reg  [5:0] counter, counter_nxt;

    assign PC_o = PC;

    //type
    parameter R = 3'b000;
    parameter I = 3'b001;
    parameter S = 3'b010;
    parameter B = 3'b011;
    parameter JAL = 3'b100;
    parameter JALR = 3'b101;
    parameter AUPIC = 3'b110;
    parameter LW = 3'b111;

    //mode
    parameter MUL = 4'b0000;
    parameter DIV = 4'b0001;
    parameter AND = 4'b0010;
    parameter OR = 4'b0011;
    parameter ADD = 4'b0100;
    parameter SUB = 4'b0101;
    parameter LT = 4'b0110;
    parameter SL = 4'b0111;
    parameter SR = 4'b1000;

    // Todo 2: Counter
    always @(*) begin
        case(mode)
            MUL : counter_nxt = counter + 5'd1;
            DIV : counter_nxt = counter + 5'd1;
            default : counter_nxt = 5'd0;
        endcase
    end

    always @(*) begin
        case(type)
            R: begin
                if (mode == MUL || mode == DIV) begin
                    if (counter < 6'd33) PC_nxt = PC;
                    else PC_nxt = PC + 32'd4; 
                end
                else PC_nxt = PC + 32'd4; 
            end
            B: PC_nxt = zero ? (PC + imm) : (PC + 4);
            JAL: PC_nxt = PC + imm;
            JALR: PC_nxt = rs1_data + imm;
            default: PC_nxt = PC + 32'd4; 
            

        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            counter <= 6'd0;
            
        end
        else begin
            PC <= PC_nxt;
            counter <= counter_nxt;
        end
    end

endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule
module BasicALU(clk, rst_n, mode, in_A, in_B, out, zero);
    input clk, rst_n;
    input [3:0] mode;
    input [31:0] in_A, in_B;
    output [31:0] out;
    output zero;

    //mode
    parameter MUL = 4'b0000;
    parameter DIV = 4'b0001;
    parameter AND = 4'b0010;
    parameter OR = 4'b0011;
    parameter ADD = 4'b0100;
    parameter SUB = 4'b0101;
    parameter LT = 4'b0110;
    parameter SL = 4'b0111;
    parameter SR = 4'b1000;

    reg [31:0] data;
    assign out = data;
    assign zero = (data == 32'd0);

    always @(*) begin
        case(mode)
            AND: data = in_A & in_B;
            OR: data = in_A | in_B;
            ADD: data = in_A + in_B;
            SUB: data = in_A - in_B;
            LT: data = (in_A < in_B) ? 32'd1: 32'b0;
            SL: data = in_A << in_B;
            SR: data = $signed(in_A) >>> in_B;
            default: data = 32'b0;
        endcase
    end


endmodule

module MulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2

    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [3:0]  mode;
    output        ready;
    input  [31:0] in_A, in_B;
    output [31:0] out;

    // Definition of states
    parameter IDLE = 2'd0;
    parameter MULI  = 2'd1;
    parameter DIVI  = 2'd2;
    parameter OUT  = 2'd3;

    // Mode parameter
    parameter MUL = 4'b0000;
    parameter DIV = 4'b0001;
    parameter AND = 4'b0010;
    parameter OR = 4'b0011;
    parameter ADD = 4'b0100;
    parameter SUB = 4'b0101;
    parameter LT = 4'b0110;
    parameter SL = 4'b0111;
    parameter SR = 4'b1000;
    // Todo: Wire and reg if needed
    reg  [ 1:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign ready = (state == OUT) ? 1'b1 : 1'b0;
    assign out = shreg[31:0];
    
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (!valid) state_nxt = IDLE;
                else case(mode)
                    MUL : state_nxt = MULI;
                    DIV : state_nxt = DIVI;
                    default: state_nxt = IDLE;
                endcase
            end
            MULI : begin
                if (counter < 5'd31) state_nxt = MULI;
                else state_nxt = OUT;
            end
            DIVI : begin
                if (counter < 5'd31) state_nxt = DIVI;
                else state_nxt = OUT;
            end
            OUT : state_nxt = IDLE;
            default : state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(*) begin
        case(state)
            MULI : counter_nxt = counter + 5'd1;
            DIVI : counter_nxt = counter + 5'd1;
            default : counter_nxt = 5'd0;
        endcase
    end
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MULI : begin
                if (!shreg[0]) alu_out = shreg[63:32];  //add 0
                else alu_out = (alu_in + shreg[63:32]); //add alu_in
            end
            DIVI : alu_out = shreg[63:31] - alu_in; //32?
            default : alu_out = 32'b0;
        endcase
    end
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            MULI : shreg_nxt = {alu_out, shreg[31:1]}; //alu_out: 33bits
            DIVI : begin
                if (alu_out[32])  shreg_nxt = {shreg[62:0], 1'b0};
                else shreg_nxt = {alu_out[31:0], shreg[30:0], 1'b1}; //30 31?
            end
            default : shreg_nxt = {32'b0, in_A};
        endcase
    end
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            counter <= 5'd0;
            alu_in <= 32'd0;
            shreg <= {32'b0, in_A};
        end
        else begin
            state <= state_nxt;
            counter <= counter_nxt;
            alu_in <= alu_in_nxt;
            shreg <= shreg_nxt;
        end
    end

endmodule

module Control(
    input clk, rst_n,
    input  [31:0] mem_rdata_I,
    output Branch_o, MemRead_o, MemToReg_o, MemWrite_o, AluSrc_o, RegWrite_o, Jalr_o, Jal_o, Aupic_o,
    output [4:0] rs1, rs2, rd,
    output [31:0] imm_o,
    output [3:0] AluOp_o, mode_o,
    output [2:0] type_o
    );
    
    //type
    parameter R = 3'b000;
    parameter I = 3'b001;
    parameter S = 3'b010;
    parameter B = 3'b011;
    parameter JAL = 3'b100;
    parameter JALR = 3'b101;
    parameter AUPIC = 3'b110;
    parameter LW = 3'b111;
    //opcode
    parameter R_op = 7'b0110011;
    parameter I_op = 7'b0010011;
    parameter S_op = 7'b0100011;
    parameter B_op = 7'b1100011;
    parameter JAL_op = 7'b1101111;
    parameter JALR_op = 7'b1100111;
    parameter AUPIC_op = 7'b0010111;
    parameter LW_op = 7'b0000011;
    //mode(AluOp)
    parameter MUL = 4'b0000;
    parameter DIV = 4'b0001;
    parameter AND = 4'b0010;
    parameter OR = 4'b0011;
    parameter ADD = 4'b0100;
    parameter SUB = 4'b0101;
    parameter LT = 4'b0110;
    parameter SL = 4'b0111;
    parameter SR = 4'b1000;

    wire [6:0] opcode;
    reg Branch, MemRead, MemToReg, MemWrite, AluSrc, RegWrite, Jalr, Jal, Aupic;
    reg [2:0] type;
    reg [3:0] AluOp, mode;
    reg [31:0] imm;
    assign Branch_o = Branch;
    assign MemRead_o = MemRead;
    assign MemToReg_o = MemToReg;
    assign MemWrite_o = MemWrite;
    assign AluSrc_o = AluSrc;
    assign RegWrite_o = RegWrite;
    assign Jalr_o = Jalr;
    assign Jal_o = Jal;
    assign Aupic_o = Aupic;
    assign type_o = type;
    assign mode_o = mode;
    assign AluOp_o = AluOp;
    
    wire [2:0] func3;
    wire [6:0] func7;
    assign rs1 = mem_rdata_I[19:15];
    assign rs2 = mem_rdata_I[24:20];
    assign rd  = mem_rdata_I[11: 7];
    assign opcode = mem_rdata_I[6:0];
    assign func3 = mem_rdata_I[14:12];
    assign func7 = mem_rdata_I[31:25];
    assign imm_o = imm;

    //always @(posedge clk or negedge rst_n or mem_rdata_I) begin
    always @(*) begin  
        case (opcode)
            R_op: begin
                type = R;
                case(func7)
                    7'b0000001: begin
                        case(func3)
                            3'b000: begin
                                AluOp = MUL;
                                mode = MUL;
                            end
                            default: begin
                                AluOp = DIV;
                                mode = DIV;
                            end
                        endcase
                    end
                    7'b0000000: begin
                        AluOp = ADD;
                        mode = ADD;
                    end
                    default: begin
                        AluOp = SUB;
                        mode = SUB;
                    end
                endcase
                imm = 32'd0;
                Branch = 1'd0;
                MemRead = 1'd0;
                MemToReg = 1'd0;
                MemWrite = 1'd0;
                AluSrc = 1'd0;
                RegWrite = 1'd1;
                Jalr = 1'd0;
                Jal = 1'd0;
                Aupic = 1'd0;
            end
            I_op: begin
                type = I;
                if(mem_rdata_I[13]) begin
                    AluOp = LT;
                    mode = LT;
                    imm[11:0] = mem_rdata_I[31:20];
                    imm[31:12] = imm[11] ? 20'b11111111111111111111 : 20'b0;
                end
                else if (mem_rdata_I[14]) begin
                    AluOp = SR;
                    mode = SR;
                    imm[4:0] = mem_rdata_I[24:20];
                    imm[31:5] = 27'b0;
                end
                else if (mem_rdata_I[12]) begin
                    AluOp = SL;
                    mode = SL;
                    imm[4:0] = mem_rdata_I[24:20];
                    imm[31:5] = 27'b0;
                end
                else  begin
                    AluOp = ADD;
                    mode = ADD;
                    imm[11:0] = mem_rdata_I[31:20];
                    imm[31:12] = imm[11] ? 20'b11111111111111111111 : 20'b0;
                end
                
                Branch = 1'd0;
                MemRead = 1'd0;
                MemToReg = 1'd0;
                MemWrite = 1'd0;
                AluSrc = 1'd1;
                RegWrite = 1'd1;
                Jalr = 1'd0;
                Jal = 1'd0;
                Aupic = 1'd0;
            end
            S_op: begin
                type = S;
                AluOp = ADD;
                mode = ADD;
                imm[4:0] = mem_rdata_I[11:7];
                imm[11:5] = mem_rdata_I[31:25];
                //imm[31:12] = imm[11] ? 20'b0: 20'b0;
                imm[31:12] = imm[11] ? 20'b11111111111111111111 : 20'b0;
                Branch = 1'd0;
                MemRead = 1'd0;
                MemToReg = 1'd0;
                MemWrite = 1'd1;
                AluSrc = 1'd1;
                RegWrite = 1'd0;
                Jalr = 1'd0;
                Jal = 1'd0;
                Aupic = 1'd0;
            end
            B_op: begin
                type = B;
                AluOp = SUB;
                mode = SUB;
                imm[11] = mem_rdata_I[7];
                imm[4:1] = mem_rdata_I[11:8];
                imm[10:5] = mem_rdata_I[30:25];
                imm[12] = mem_rdata_I[31];
                imm[0] = 1'b0;
                imm[31:13] = imm[12] ? 19'b1111111111111111111 : 19'b0;
                Branch = 1'd1;
                MemRead = 1'd0;
                MemToReg = 1'd0;
                MemWrite = 1'd0;
                AluSrc = 1'd0;
                RegWrite = 1'd0;
                Jalr = 1'd0;
                Jal = 1'd0;
                Aupic = 1'd0;
            end
            JAL_op: begin
                type = JAL;
                AluOp = ADD;
                mode = ADD;
                imm[19:12] = mem_rdata_I[19:12];
                imm[11] = mem_rdata_I[20];
                imm[10:1] = mem_rdata_I[30:21];
                imm[20] = mem_rdata_I[31];
                imm[0] = 1'b0;
                imm[31:21] = mem_rdata_I[31] ? 11'b11111111111 : 11'd0;
                Branch = 1'd0;
                MemRead = 1'd0;
                MemToReg = 1'd0;
                MemWrite = 1'd0;
                AluSrc = 1'd1;
                RegWrite = 1'd1;
                Jalr = 1'd0;
                Jal = 1'd1;
                Aupic = 1'd0;
            end
            JALR_op: begin
                type = JALR;
                AluOp = ADD;
                mode = ADD;
                imm[11:0] = mem_rdata_I[31:20];
                imm[31:12] = imm[11] ? 20'b11111111111111111111 : 20'b0;
                Branch = 1'd0;
                MemRead = 1'd0;
                MemToReg = 1'd0;
                MemWrite = 1'd0;
                AluSrc = 1'd1;
                RegWrite = 1'd1;
                Jalr = 1'd1;
                Jal = 1'd0;
                Aupic = 1'd0;
            end
            AUPIC_op: begin
                type = AUPIC;
                AluOp = ADD;
                mode = ADD;
                imm[31:12] = mem_rdata_I[31:12];
                imm[11:0] = 12'b0;
                Branch = 1'd0;
                MemRead = 1'd0;
                MemToReg = 1'd0;
                MemWrite = 1'd0;
                AluSrc = 1'd1;
                RegWrite = 1'd1;
                Jalr = 1'd0;
                Jal = 1'd0;
                Aupic = 1'd1;
            end
            LW_op: begin
                type = LW;
                AluOp = ADD;
                mode = ADD;
                imm[11:0] = mem_rdata_I[31:20];
                imm[31:12] = imm[11] ? 20'b11111111111111111111 : 20'b0;
                Branch = 1'd0;
                MemRead = 1'd1;
                MemToReg = 1'd1;
                MemWrite = 1'd0;
                AluSrc = 1'd1;
                RegWrite = 1'd1;
                Jalr = 1'd0;
                Jal = 1'd0;
                Aupic = 1'd0;
            end
            default:begin
                type = 3'd0;
                AluOp = ADD;
                mode = ADD;
                imm[11:0] = mem_rdata_I[31:20];
                imm[31:12] = 20'b0;
                Branch = 1'd0;
                MemRead = 1'd0;
                MemToReg = 1'd0;
                MemWrite = 1'd0;
                AluSrc = 1'd0;
                RegWrite = 1'd0;
                Jalr = 1'd0;
                Jal = 1'd0;
                Aupic = 1'd0;
            end
        endcase
    end
endmodule


