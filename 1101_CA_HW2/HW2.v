module ALU(
    clk,
    rst_n,
    valid,
    ready,
    mode,
    in_A,
    in_B,
    out
);

    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: and, 3: or
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter AND = 3'd3;
    parameter OR = 3'd4;
    parameter OUT  = 3'd5;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign ready = (state == OUT) ? 1'b1 : 1'b0;
    assign out = shreg;
    
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (!valid) state_nxt = IDLE;
                else case(mode)
                    2'd0 : state_nxt = MUL;
                    2'd1 : state_nxt = DIV;
                    2'd2 : state_nxt = AND;
                    2'd3 : state_nxt = OR;
                endcase
            end
            MUL : begin
                if (counter < 5'd31) state_nxt = MUL;
                else state_nxt = OUT;
            end
            DIV : begin
                if (counter < 5'd31) state_nxt = DIV;
                else state_nxt = OUT;
            end
            AND : state_nxt = OUT;
            OR  : state_nxt = OUT;
            OUT : state_nxt = IDLE;
            default : state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(*) begin
        case(state)
            MUL : counter_nxt = counter + 5'd1;
            DIV : counter_nxt = counter + 5'd1;
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
            MUL : begin
                if (!shreg[0]) alu_out = shreg[63:32];	//add 0
                else alu_out = (alu_in + shreg[63:32]);	//add alu_in
            end
            DIV : alu_out = shreg[63:31] - alu_in; //32?
            AND : alu_out = shreg[31:0] & alu_in;
            OR : alu_out = shreg[31:0] | alu_in;
            default : alu_out = 32'b0;
        endcase
    end
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            MUL : shreg_nxt = {alu_out, shreg[31:1]}; //alu_out: 33bits
            DIV : begin
                if (alu_out[32])  shreg_nxt = {shreg[62:0], 1'b0};
                else shreg_nxt = {alu_out[31:0], shreg[30:0], 1'b1}; //30 31?
            end
            AND : shreg_nxt = {32'b0, alu_out}; 
            OR : shreg_nxt = {32'b0, alu_out};
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