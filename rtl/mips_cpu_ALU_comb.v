module comparator (
    /*inputs:a, b, 
pos=a single number positive or not?, 
neg= a single number negative or not?, 
eql = a is equal to b, 
zero = a is equal to 0; 
outputs:met*/
    input logic[31:0] a,
    input logic[31:0] b,
    input logic pos,
    input logic neg,
    input logic eql,
    input logic zero,
    input logic neq,
    input logic[31:0] r,
    
    
    // connects to alu
    output logic[31:0] aluA,
    output logic[31:0] aluB,
    output logic[5:0] opcode,
    output logic met
);

    logic[31:0] c_0;
    logic[31:0] acc;

    
    assign c_0 = 0;
        
    always @(*) begin
        met = 0;
        if(pos)begin
            if((a[31]==0)&&(a!=0))begin
                met=1;
            end
        end 
        if(neg) begin
            if(a[31]==1)begin
                met=1;
            end
        end 
        if(eql||neq) begin
            // outs to ALU
            aluA = a;
            aluB = b;
            opcode = 6'b100011;//sub-opcode
            // ALU response
            acc = r;
            if(((acc==0)&&(eql==1))||((acc!=0)&&(neq==1))) begin
                met = 1;
            end
        end 
        if (zero) begin
            // outs to ALU
            if(a[31:0]==0)begin
                met = 1;
            end
        end 

    end
    
endmodule

module HILO (
    input clk,
    input logic[31:0] LO,
    input logic[31:0] HI,
    output logic[31:0] LO_out,
    output logic[31:0] HI_out
);
    
    always_ff @(posedge clk) begin
        LO_out <= LO;
        HI_out <= HI;
    end

endmodule

//2 operands, opcode(6 bits), check conans 
module alu_op ( 
    input logic[31:0] a,
    input logic[31:0] b,
    input logic[5:0] opcode,
    input logic[4:0] shamt,
    input logic RESET,
    output logic[31:0] r,
    output logic[31:0] LO,
    output logic[31:0] HI
);
    logic [63:0] product;

    always @(*) begin
        if(RESET)begin
            LO = 0;
            HI = 0;
        end 
        else begin    
            //$display("ALU: GEN ALU opcode:",opcode);
            case (opcode)
                //ADDIU, ADDU
                6'b001001,6'b100001: begin //extract da immediate 
                    r = a + b;
                    // $display("ALU: ADD ALU output:",r);
                    // $display("ALU: ADD ALU input A:",a);
                    // $display("ALU: ADD ALU input B:",b);
                    // $display("ALU: ADD ALU opcode:",opcode);
                end
                //AND, ANDI
                6'b100100,6'b001100: begin
                    r = a & b;
                    // $display("ALU: AND ALU output:",r);
                    // $display("ALU: AND ALU input A:",a);
                    // $display("ALU: AND ALU input B:",b);
                    // $display("ALU: AND ALU opcode:",opcode);
                end
                //DIV, DIVU
                6'b011010,6'b011011: begin // quotient in LO, rem in HI
                    LO = a / b;
                    HI = a % b; 
                end
                //MFLO
                6'b010010: begin 
                    r = LO; 
                end
                //MFHI
                6'b010000: begin 
                    r = HI;
                end
                //MULT, MULTU
                6'b011000,6'b011001: begin // first 32 bits in hi last in lo
                    product = a * b;
                    LO = product[31:0];
                    HI = product[63:32];
                end
                //MTLO
                6'b010011:begin
                    LO = a; //First operand goes into the register LO. Second operand is ignored.
                end
                //MTHI
                6'b010001: begin
                    HI = a; //First operand goes into the register LO. Second operand is ignored.
                end
                //OR, ORI
                6'b100101,6'b001101: begin
                    r = a | b;
                end
                //SLLV
                6'b000100: begin //always shift by b. inputs into the ALU are either the register value or an immediate
                    //value from register
                    r = a << b; 
                end
                //SLL
                6'b000000: begin //immediate version
                    r = a << shamt;
                end
                //SRA
                6'b000011: begin
                    r = a >>> shamt;
                end
                //SRAV
                6'b000111: begin
                    //value from register
                    r = a >>> b;
                end
                //SRL
                6'b000010: begin
                    r = a >> shamt;
                end
                //SRLV
                6'b000110: begin
                    //value from register
                    r = a >> b;
                end
                //SLT (Set on less than (signed)), SLTI (Set on less than immediate (signed)), SLTIU (Set on less than immediate unsigned), SLTU (Set on less than unsigned)
                6'b101010, 6'b001010, 6'b001011, 6'b101011: begin                    
                    r = a < b; //a = rs, b = rt
                end
                //SUBU
                6'b100011: begin
                    r = a - b;
                end
                //XOR, XORI
                6'b100110, 6'b001110: begin
                    r = a ^ b;
                end
                
                default: begin //else basc
                    r = 0;
                end
            endcase
             
        end
    end
    

endmodule

module ALU_comb (
    input   logic       clk,
    input   logic       pos,
    input   logic       neg,
    input   logic       eql,
    input   logic       neq,
    input   logic       zero,
    input   logic       reset,
    input   logic[5:0]  opcode,
    input   logic[4:0]  shamt,
    input   logic[31:0] a,
    input   logic[31:0] b,
    output  logic[31:0] r,
    output  logic       comp_met,
    output  logic[31:0] HI_o,
    output  logic[31:0] LO_o
);

    logic[31:0] HI,LO;
    logic[31:0] alu_out;
    logic[5:0]  alu_opcode, compOp;
    logic[31:0] alu_in_a, alu_in_b, compA, compB;

    // if pos, neg, eql or zero is high, use opcodes from comp, else use op_in
    // logic to determine alu_in (either from comp or external)
    alu_op ALU(
        .a(alu_in_a),
        .b(alu_in_b),
        .opcode(alu_opcode),
        .shamt(shamt),
        .RESET(reset),
        .r(alu_out),
        .LO(LO),
        .HI(HI)
    );
    // comp can be a and b bc no external block using comp, aluA, aluB, opcode need different wire
    comparator COMP(
        .a(a),
        .b(b),
        .pos(pos),
        .neg(neg),
        .eql(eql),
        .neq(neq),
        .zero(zero),
        .r(alu_out),

        .aluA(compA),//
        .aluB(compB),//
        .opcode(compOp),//
        .met(comp_met) //
    );

    HILO H_L(
        .clk(clk),
        .LO(LO),
        .HI(HI),
        .LO_out(LO_o),
        .HI_out(HI_o)  
    );

    always @(posedge alu_out) begin
        $display("ALU: ALU input A:",alu_in_a);
        $display("ALU: ALU input B:",alu_in_b);
        $display("ALU: ALU opcode:",alu_opcode);
        $display("ALU: ALU r:",alu_out);
    end
    //From here//
    //commands the whole block which consists of: ALU, Comparator, LOHI regs
    always_comb begin
        if((pos == 1)||(neg == 1)||(eql == 1)||(zero == 1)||(neq==1)) begin 
            // if pos, neg, eql or zero is high, use opcodes from comp, else use opcode. Also change the a/b inputs into the ALU. 
            alu_opcode = compOp;
            alu_in_a = compA;
            alu_in_b = compB;
        end
        else begin
            alu_in_a = a;
            alu_in_b = b;
            alu_opcode = opcode;
            r = alu_out;
            //not sure what to do with LO_o and HI_o. Maybe psuedocode-wise it would look like this:
            
        end
    end

endmodule