module ControlUnit(
    output logic[5:0] OPCODE, //
    input logic waitrequest, //
    input logic clock,
    input logic reset,
    output logic RAMreadReq,

    //State Machine
    //input logic FETCH, //
    //input logic EXEC1, // 
    //input logic EXEC2, //
    output logic ACTIVE,
    output logic HALT, //
    output logic STALL,

    //PC
    //input logic[31:0] PC, //
    output logic[31:0] PCOffset, //

    //RAM
    input logic[31:0] RAMDATA, //
    output logic[31:0] RAMADDR, //
    output logic RAMWRITE, //SPECIAL //
    output logic[31:0] RAMOUT, //

    //RegFile
    output logic[4:0] Rs, //
    output logic[4:0] Rt, //
    output logic[4:0] Rd, //
    //input logic[31:0] RsDATA, //
    //input logic[31:0] RtDATA, //
    output logic[31:0] regv0, //
    output logic WENREG, //SPECIAL //

    //ALU
    output logic[31:0] OP1, //
    output logic[31:0] OP2, //
    output logic[4:0] SHAMT, //
    
    //input logic[31:0] RESULT, //

    //IR Block
    //input logic[5:0] op_code, //Changes location depending on instruction type
    //input logic Rtype, //
    //input logic Itype, //
    //input logic Jtype, //

    //input logic isArith, //
    //input logic isBranch, //
    //input logic isLink, //

    //input logic CONDITIONMET, //
    //input logic P, //
    //input logic N, //
    //input logic EQ, //
    //input logic NEQ, //
    //input logic Z, //

    //LS Block
    //input logic[31:0] LSRAMADDR, //
    output logic[3:0] byteEnable, //
    output logic[31:0] LSRAMIN //
    //input logic isLoad, //
    //input logic isStore, //

   
);
    //State Machine
    logic FETCH, EXEC1, EXEC2; //ACTIVE;

    //PC
    logic[31:0] PC;
    logic updatePC;

    //RegFile
    logic[31:0] RsDATA, RtDATA; //regv0;

    //ALU
    logic[5:0] ALUCODE;
    logic[31:0] RESULT;
    logic CONDITIONMET;

    //IR Block
    // OPCODE is an output ?
    logic[5:0] FUNCCODE;
    logic[4:0] RT;
    logic Rtype, Itype, Jtype, isArith, isBranch, isLink, P, N, EQ, NEQ, Z, signReqd;
    logic[5:0] throw_away_opcode;

    //Decode instr. word
    logic[15:0] Immediate;
    logic[25:0] Target;
    logic jump_r, jump_const;
    logic[31:0] jump_reg_value;
    
    //LS Block
    logic[31:0] LSRAMADDR;
    logic[31:0] RdDATA; //

    logic[31:0] IW;
    logic[31:0] regData;
    logic waitreqflag;
    //assign LSIW = IW;
    
    // logic delay_halt_1, delay_halt_2;
    // logic[31:0] delay_offset;
    // logic RAM_read_data;
    // logic[31:0] LSRAMOUT;
    
    assign waitreqflag = waitrequest && (RAMreadReq || RAMWRITE); //condition for waitreq to stall the cpu
    assign RAMreadReq = RAM_read_data || FETCH;
    // assign IW = {RAMDATA[7:0], RAMDATA[15:8], RAMDATA[23:16], RAMDATA[31:24]}; //Endian conversion
    
  always @(*) begin
        
    if (FETCH)begin
        //WENREG = 0;
        //PCOffset = 0;
        $display("CTRL: Here is the result = %d",RESULT);
        RAMADDR = PC; //Fetch instruction from PC
        // $display("CTRL: Is the CPU active right now?: ", ACTIVE);
        $display("CTRL: Read instruction at RAM address: %h", RAMADDR);
        // if ((IW[31:26] == 6'b000000) && (IW[5:0] == 6'b001000)) begin
        //     HALT = 1;
        // end
        if(PC == 32'h00000000) begin
            HALT = 1;
        end
        WENREG = 1;
        updatePC = 0;
        
    end
    //-----------------------------------------------------------------------------------------------------------------------
    if (EXEC1)begin
        $display("CTRL: RAM INFO DATA READ: %h", RAMDATA);
        WENREG = 0;
        updatePC = 0;
        IW = {RAMDATA[7:0], RAMDATA[15:8], RAMDATA[23:16], RAMDATA[31:24]};
        OPCODE = IW[31:26];
        FUNCCODE = IW[5:0];
        Rs = IW[25:21];
        // RT = IW[20:16];

        //$display("CTRL: Data received from RAM: %h", RAMDATA);
        $display("CTRL : instruction word: %h, opcode: %b, funccode: %b", IW, OPCODE, FUNCCODE);
        // OPCODE = IW[31:26];
        
        // RT = IW[20:16];
       //Is instruction R, I, or J type?
        if (OPCODE == 6'b000000) begin
            $display("RRRRRRRRRR type");
            // Rtype = 1;
            // FUNCCODE = IW[5:0];
        end
        else if ((OPCODE == 6'b000010) || (OPCODE == 6'b000011)) begin
            // Jtype = 1;
            $display("JJJJJJJJJJJ type");
            
        end
        else begin
            // Itype = 1;
            // FUNCCODE = IW[5:0];
            $display("IIIIIIIIII type");
            // if (((OPCODE == 6'b000001)&&(RT == 5'b00001)) || ((OPCODE == 6'b000001)&&(RT == 5'b10001))||((OPCODE == 6'b000001)&&(RT == 5'b00000))||((OPCODE == 6'b000001)&&(RT == 5'b10000))) begin
            //     RT = IW[20:16];
            // end
            // else begin
            //     FUNCCODE = IW[5:0];
            // end
        end
        $display("DECODE: Decoding is done, enterring parameter assginment");

        // r-type expressions
        if(Rtype)begin
            // ADDU, AND, OR, SLLV, SRAV, SRLV, SUBU, XOR, SLT, SLTU
            if((FUNCCODE == 6'b100001)||(FUNCCODE == 6'b100100)||(FUNCCODE == 6'b100101)||(FUNCCODE == 6'b000100)||(FUNCCODE == 6'b000111)||(FUNCCODE == 6'b000110)||(FUNCCODE == 6'b100011)||(FUNCCODE == 6'b100110)||(FUNCCODE == 6'b101010)||(FUNCCODE == 6'b101011))begin
                Rs = IW[25:21];
                Rt = IW[20:16];
                Rd = IW[15:11];
                SHAMT = 0;
                ALUCODE = FUNCCODE;
            end
            // DIV, DIVU, MULT, MULTU
            else if((FUNCCODE == 6'b011010)||(FUNCCODE == 6'b011011)||(FUNCCODE == 6'b011000)||(FUNCCODE == 6'b011001))begin
                Rs = IW[25:21];
                Rt = IW[20:16];
                Rd = 0;
                SHAMT = 0;
                ALUCODE = FUNCCODE;
            end
            //SLL, SRA, SRL
            else if((FUNCCODE == 6'b000000)||(FUNCCODE == 6'b000011)||(FUNCCODE == 6'b000010)) begin
                Rs = 0;
                Rt = IW[20:16];
                Rd = IW[15:11];
                SHAMT = IW[10:6];
                ALUCODE = FUNCCODE;
                $display("CTRL: SLL SHAMT %h, Instruction:%h, FUNCCODE:%b",SHAMT, IW, FUNCCODE);
                $display("HHHHHHHHH ALU CODE %h", ALUCODE);
            end
            //JR, MTHI, MTLO
            else if((FUNCCODE == 6'b001000)||(FUNCCODE == 6'b010001)||(FUNCCODE == 6'b010011))begin
                Rs = IW[25:21];
                Rt = 0;
                Rd = 0;
                SHAMT = 0;
                ALUCODE = FUNCCODE;
                $display("Detect JR in EXEC 1, Rs: %d", Rs);
            end
            //JALR
            else if(FUNCCODE == 6'b001001)begin
                Rs = IW[25:21];
                Rt = 0;
                Rd = IW[15:11];
                SHAMT = 0;
                ALUCODE = 6'b111111;
            end
            // MFHI, MFLO
            else if((FUNCCODE == 6'b010000)||(FUNCCODE == 6'b010010))begin
                Rs = 0;
                Rt = 0;
                Rd = IW[15:11];
                SHAMT = 0;
                ALUCODE = FUNCCODE;
            end  
        end
        //I-Type
        else if(Itype)begin
            //BGEZ, BGEZAL, BLTZ, BLTZAL 
            if((OPCODE == 6'b000001))begin
                if((RT == 5'b00001)||(RT == 5'b10001)||(RT == 5'b00000)||(RT == 5'b10000))begin
                    Rs = IW[25:21];
                    Rt = 0;
                    Immediate = IW[15:0];
                end
                ALUCODE = 6'b111111;
            end
            //BGTZ, BLEZ
            else if((OPCODE == 6'b000111)||(OPCODE == 6'b000110)) begin
                Rs = IW[25:21];
                Rt = 0;
                Immediate = IW[15:0];
                ALUCODE = 6'b111111;
            end
            //ADDIU, ANDI, ORI, XORI, BEQ, BNE, SLTI, SLTIU
            // no clash of instr. Comparator overrides
            else if((OPCODE == 6'b001001)||(OPCODE == 6'b001100)||(OPCODE == 6'b001101)||(OPCODE == 6'b001110)||(OPCODE == 6'b001010)||(OPCODE == 6'b001011)||(OPCODE == 6'b000100)||(OPCODE == 6'b000101))begin
                Immediate = IW[15:0];
                Rs = IW[25:21];
                Rt = IW[20:16];
                ALUCODE = OPCODE;
                Rd = Rt;
                $display("CTRL: INFO: ADDIU called");
                $display("CTRL: INPUT INFO: Rs = %h Rt = %h Immediate = %h ALUCODE = %h Rd = %h",IW[25:21],IW[20:16],IW[15:0],OPCODE,Rs);
            end
            //LB, LBU, LH, LHU, LW, LWL, LWR, SB, SH, SW
            else if((OPCODE == 6'b100000)|| (OPCODE == 6'b100100) || (OPCODE == 6'b100001) || (OPCODE == 6'b100101) || 
            (OPCODE == 6'b100011) || (OPCODE == 6'b101000) || (OPCODE == 6'b101001) || (OPCODE == 6'b101011) || 
            (OPCODE == 6'b100010) || (OPCODE == 6'b100110)) begin
                Rs = IW[25:21];
                Rt = IW[20:16];
                Immediate = IW[15:0];

                if (OPCODE[5:3] == 3'b100) begin
                    RAMADDR = LSRAMADDR;
                end 

                ALUCODE = 6'b111111; // opcode that ensures default case in alu
            end
            //LUI
            else if(OPCODE == 6'b001111)begin
                Rs=0;
                Rt = IW[20:16];
                Immediate = IW[15:0]; 
                ALUCODE = 6'b111111; // opcode that ensures default case in alu
            end
        end //end for itype
        else if(Jtype) begin 
            Target = IW[25:0];   
        end

        if(isLink) begin
            if(FUNCCODE == 6'b001001) begin //JALR
                Rd = IW[15:11];
            end 
            else begin
                Rd = 31;
            end
        end
        
        // $display("CTRL: Rd is %d",  Rd);
        // $display("CTRL: Rs is %d",  Rs);
        // $display("CTRL: Rt is %d",  Rt);
        
    end
    //--------------------------------------------------------------------------------------------------------
    if (EXEC2) begin
        jump_r = 0;
        jump_const = 0;
        WENREG = 0;
        updatePC = !waitreqflag;

        $display("CTRL: Instruction again: %b, FUNCCODE: %b", IW, FUNCCODE);
        //need to decode the instructions according to the sytle in the assembler
        if(Rtype)begin
            // ADDU, AND, OR, SRLV, SUBU, XOR, SLT, SLTU
            if((FUNCCODE == 6'b100001)||(FUNCCODE == 6'b100100)||(FUNCCODE == 6'b100101)||(FUNCCODE == 6'b000110)||(FUNCCODE == 6'b100011)||(FUNCCODE == 6'b100110)||(FUNCCODE == 6'b101010)||(FUNCCODE == 6'b101011))begin
                OP1 = RsDATA; //Set Rs
                OP2 = RtDATA; //Set Rt
                regData = RESULT; //Save result
                //$display("CTRL: Here is the result = %d",RESULT);
                //$display("CTRL: printing the result now: ", RESULT);
                WENREG = 1;
            end
            // SLLV, SRAV
            else if((FUNCCODE == 6'b000100)||(FUNCCODE == 6'b000111))begin
                OP1 = RtDATA; //Set Rt
                OP2 = RsDATA; //Set Rs
                regData = RESULT; //Save result
                //$display("CTRL: printing the result now: ", RESULT);
                WENREG = 1;
            end
            // DIV, DIVU, MULT, MULTU
            // contents in HI,LO
            else if((FUNCCODE == 6'b011010)||(FUNCCODE == 6'b011011)||(FUNCCODE == 6'b011000)||(FUNCCODE == 6'b011001))begin
                OP1 = RsDATA; //Set Rs
                OP2 = RtDATA; //Set Rt
                //$display("CTRL: printing the result now: ", RESULT);
            end
            //SLL, SRA, SRL
            else if((FUNCCODE == 6'b000000)||(FUNCCODE == 6'b000011)||(FUNCCODE == 6'b000010)) begin
                OP1 = RtDATA; //Set Rt
                OP2 = 0; //Not required
                regData = RESULT; //Save result
                $display("CTRL: INFO: Currently in the SLL statement");
                WENREG = 1;
            end
            //MTHI, MTLO
            else if((FUNCCODE == 6'b010001)||(FUNCCODE == 6'b010011))begin
                OP1 = RsDATA; //Set Rs
                OP2 = 0; //Not requireds
            end
            // MFHI, MFLO
            else if((FUNCCODE == 6'b010000)||(FUNCCODE == 6'b010010))begin
                OP1 = 0; //Not required
                OP2 = 0; //Not required
                //$display("CTRL: printing the result now: ", RESULT);
                regData = RESULT; //Save result
                WENREG = 1;
            end 
            //JR
            else if(FUNCCODE == 6'b001000)begin
                $display("Arrived at jr: %h, Rs: %d", RsDATA, Rs);
                OP1 = 0; //Not required
                OP2 = 0; //Not required
                jump_r = 1;
                jump_reg_value = RsDATA;                
            end    
            //JALR
            else if(FUNCCODE == 6'b001001)begin
                OP1 = 0; //Not required
                OP2 = 0; //Not required
                jump_r = 1;
                WENREG = 1;
                regData = PC;
                jump_reg_value = RsDATA;                
            end
             
        end
        //I-Type
        else if(Itype) begin
            //BGEZ, BGEZAL, BLTZ, BLTZAL 
            if((OPCODE == 6'b000001))begin
                if((RT == 5'b00001)||(RT == 5'b10001)||(RT == 5'b00000)||(RT == 5'b10000)) begin
                    if (CONDITIONMET) begin
                        PCOffset = Immediate;
                        jump_const = 1;
                        
                        if(isLink) begin
                            regData = PC;
                            WENREG = 1;
                        end
                    end
                end
            end
            //BGTZ, BLEZ, BEQ, BNE
            else if(((OPCODE == 6'b000111)||(OPCODE == 6'b000110)) || ((OPCODE == 6'b000100)||(OPCODE == 6'b000101))) begin
                if (CONDITIONMET) begin
                    PCOffset = Immediate;
                    jump_const = 1;
                end
            end
            //ADDIU, ANDI, ORI, XORI, SLTI, SLTIU
            // no clash of instr. Comparator overrides
            else if((OPCODE == 6'b001001)||(OPCODE == 6'b001100)||(OPCODE == 6'b001101)||(OPCODE == 6'b001110)||(OPCODE == 6'b001010)||(OPCODE == 6'b001011))begin
                if((OPCODE == 6'b001001)||(OPCODE == 6'b001010)||(OPCODE == 6'b001011))begin
                    OP2 = $signed(Immediate);
                end
                else begin
                    OP2 = $unsigned(Immediate); //Set to immediate   
                end
                
                OP1 = RsDATA; //Set Rs
                
                regData = RESULT; //Save result
                $display("CTRL: Here is the RsDATA in andi = %d", RsDATA);
                //$display("CTRL: Here is the FUNCCODE in andi = %d", FUNCCODE);
                //$display("CTRL: printing the result now: ", RESULT);
                WENREG = 1;
            end
            //LB, LBU, LH, LHU, LW, LWL, LWR, SB, SH, SW
            else if((OPCODE == 6'b100000)|| (OPCODE == 6'b100100) || (OPCODE == 6'b100001) || (OPCODE == 6'b100101) || 
                (OPCODE == 6'b100011) || (OPCODE == 6'b101000) || (OPCODE == 6'b101001) || (OPCODE == 6'b101011) || 
                (OPCODE == 6'b100010) || (OPCODE == 6'b100110)) begin
                
                OP1 = 0; //not required
                OP2 = 0; //not required
                if (OPCODE[5:3] == 3'b100) begin
                    regData = RdDATA;
                    Rd = Rt;
                    WENREG = 1;
                end 
                else if (OPCODE[5:3] == 3'b101) begin
                    RAMADDR = LSRAMADDR;
                    updatePC = 1;
                end
                
            end
            //LUI
            else if(OPCODE == 6'b001111)begin
                Rd = Rt;
                regData = RdDATA;
                WENREG = 1;
            end
        else if(Jtype) begin 
            PCOffset = Target;
        end

        //$display("CTRL: Here is the result = %d",RESULT);
        // $display("CTRL: RegData is %h", regData);
        // $display("CTRL: Rd is supposed to be %h", RESULT);
    end

    $display("CTRL: Here is the result = %d",RESULT);
    end
  end
    

    //Instantiate modules
    statemac cpuSM(
        //Outputs to state machine
        .clk(clock),
        .halt(HALT),
        .waitrequest(waitreqflag),
        .reset(reset),

        //Inputs from state machine
        .f(FETCH),
        .e1(EXEC1),
        .e2(EXEC2),
        .active(ACTIVE)
    );


    IR cpuIR(
        //Outputs to IR
        .IW(IW),
        //Inputs from IR
        .Rtype(Rtype),
        .Itype(Itype),
        .Jtype(Jtype),
        .isArith(isArith),
        .isBranch(isBranch),
        .link(isLink),
        .P(P),
        .N(N),
        .EQ(EQ),
        .NEQ(NEQ),
        .Z(Z),
        //.OPCODE(throw_away_opcode),
        //.FUNCCODE(FUNCCODE),
        .RT(RT)
    );

    ALU_comb cpuALU(
        //Outputs to ALU
        .clk(clock),
        .reset(reset),
        .a(OP1),
        .b(OP2),
        .shamt(SHAMT),
        .opcode(ALUCODE),
        .pos(P),
        .neg(N),
        .eql(EQ),
        .neq(NEQ),
        .zero(Z),

        //Inputs from ALU
        .r(RESULT),
        .comp_met(CONDITIONMET)
    );

    registerfile cpuRegFile(
        //Outputs to regFile
        .clk(clock),
        .Rs(Rs),
        .Rt(Rt),
        .Rd(Rd),
        .WENREG((WENREG && !waitreqflag)),
        .reset(reset),

        //Inputs from regFile
        .RsDATA(RsDATA),
        .RtDATA(RtDATA),
        .RdDATA(regData),
        .register_v0(regv0)
    );

    pc cpuPC(
        //Outputs to PC
        .clk(clock),
        .reset(reset),
        .halt(HALT),
        .PCOffset(PCOffset),
        .updatePC(updatePC),
        .reg_jump_value(jump_reg_value),
        .jump_r(jump_r),
        .jump_const(jump_const),

        //Inputs from PC
        .PC(PC)
    );

    loadandstore cpuLAS(
        //Outputs to LAS
        .cycle(EXEC1),
        .fetch(FETCH), 
        .reg_s(RsDATA),
        .reg_t(RtDATA),
        .mem_read(RAMDATA),
        .instruction(IW),

        //Inputs from LAS
        .byteenable(byteEnable),
        .reg_write(RdDATA),
        .mem_write(LSRAMIN),
        //output logic reg_enable,
        //output logic[4:0] reg_write_index,
        .mem_address(LSRAMADDR), 
        .read_enable(RAM_read_data), 
        .write_enable(RAMWRITE)
    );
endmodule