module ControlUnit(
    output logic[5:0] OPCODE, //
    input logic waitrequest, //
    input logic clock,
    input logic RESET,
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
    output logic updatePC, //


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
    output logic[31:0] RdDATA, //

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
    output logic[31:0] LSRAMIN, //
    //input logic isLoad, //
    //input logic isStore, //

    output logic[31:0] LSIW, //
    output logic[31:0] LSRSDATA, //
    output logic[31:0] LSRTDATA, //
    output logic[31:0] LSRAMOUT //


   
);
    //State Machine
    logic FETCH, EXEC1, EXEC2; //ACTIVE;

    //PC
    logic[31:0] PC;

    //RegFile
    logic[31:0] RsDATA, RtDATA; //regv0;

    //ALU
    logic[31:0] RESULT;
    logic CONDITIONMET;

    //IR Block
    // OPCODE is an output ?
    logic[5:0] FUNCCODE;
    logic[4:0] RT;
    logic Rtype, Itype, Jtype, isArith, isBranch, isLink, P, N, EQ, NEQ, Z, signReqd;

    //LS Block
    logic[31:0] LSRAMADDR;
    logic isLoad, isStore;

    logic[31:0] IW;
    logic[31:0] regData;
    logic waitreqflag;
    assign LSIW = IW;
    assign waitreqflag = waitrequest && (RAMreadReq || RAMWRITE); //condition for waitreq to stall the cpu
    logic delay_halt_1, delay_halt_2;
    logic[31:0] delay_offset;

    initial begin 
        delay_halt_1 = 0;
        delay_halt_2 = 0;
        delay_offset = 0;

        WENREG = 0;
        updatePC = 0;
        PCOffset = 0;
        RAMWRITE = 0;
        RAMreadReq = 0;
    end

    always @(posedge clock)begin
        if (FETCH) begin
            // if (delay_halt_2 == 1) begin
                HALT = delay_halt_2;
                delay_halt_2 = 0;
            //end
            if (delay_halt_1 == 1) begin
                delay_halt_2 = delay_halt_1;
                delay_halt_1 = 0;
            end
        end
        else if (EXEC2 == 1) begin 
            if (delay_offset != 0) begin
                PCOffset = delay_offset;
            end
        end
    end

    always @(posedge FETCH)begin
        
        RAMADDR = PC; //Fetch instruction from PC
        $display("CTRL: Read instruction at RAM address: %h", RAMADDR);
        RAMreadReq = 1;
        
    end

    always @(posedge EXEC1)begin
        
        RAMreadReq = 0;
        WENREG = 0;
        
        //IW = {RAMDATA[24:31], RAMDATA[16:23], RAMDATA[8:15], RAMDATA[0:7]}; //Endian conversion
        IW = {RAMDATA[7:0], RAMDATA[15:8], RAMDATA[23:16], RAMDATA[31:24]};
        $display("CTRL: Data received from RAM: %h", RAMDATA);
        $display("CTRL : instruction word: %h", IW);
        // IW = RAMDATA;
        if (Rtype) begin //R type instructions
            Rt = IW[20:16];
            Rs = IW[25:21];
        end
        if (Itype) begin //I type instructions
            Rs = IW[25:21];
            if (isLoad) begin
                RAMreadReq = 1;
                RAMADDR = LSRAMADDR;
            end
        end
        if (Jtype) begin //J type instructions
            Rs = 31;
        end
    end

    always @(posedge EXEC2) begin
        //need to decode the instructions according to the sytle in the assembler
        RAMreadReq = 0;
        updatePC = 1;
        

        

        if (Rtype) begin //R type instructions
            $display("CTRL : got inside the r-type.");
            OP1 = RsDATA; //Set Rs
            OP2 = RtDATA; //Set Rt
            SHAMT = IW[10:6];
            Rd = IW[15:11];
            regData = RESULT; //Save result
            $display("CTRL: printing the result now: ", RESULT);
            WENREG = 1;

            if (IW[5:0] == 6'b001000) begin 
                delay_halt_1 = 1;
                $display("CTRL : got inside the jr part.");
            end
        end


        if (Itype) begin  //Immediate Instructions
            $display("CTRL : got inside the i-type.");
            if (isArith) begin //Arithmetic immediate instructions
                $display("in arithmetic");
                OP1 = RsDATA;
                if(signReqd) begin
                    OP2 = {16'b1111111111111111, IW[15:0]};
                end else begin
                    OP2 = {16'b0, IW[15:0]};
                end
                SHAMT = 0;
                Rd = IW[15:11];
                regData = RESULT; //Save result
                $display("CTRL: printing the result now: ", RESULT);
                WENREG = 1;
                $display("CTRL: WENREG is now high");
                end
            if (isBranch) begin //Branch instructions
                $display("CTRL : got inside the branch.");
                if (CONDITIONMET) begin 
                    // PCOffset = {14'b0, IW[15:0] << 2}; //Calc PC offset
                    // PCOffset = {14'b0, IW[15:0], 2'b0}; //Calc PC offset
                    $display("CTRL : got inside the branch. ConditionMet: %h, EQ: %h", CONDITIONMET, EQ);
                    delay_offset = {14'b0, IW[15:0], 2'b0};

                    if (isLink) begin //If link instruction save PC into register 31
                        WENREG = 1;
                        Rd = 31;
                        regData = PC;
                        end
                    end
                end
            if (isLoad) begin //Load instructions
                $display("CTRL : in the load");
                LSRSDATA = RsDATA; //Send RS data to LS
                LSRAMOUT = RAMDATA; //Send RAM data to LS
                Rd = IW[20:16]; //Save contents into register
                WENREG = 1;
                end
            if (isStore) begin //Store instructions
                $display("CTRL : in the store");
                LSRSDATA = RsDATA;
                LSRTDATA = RtDATA;
                RAMADDR = LSRAMADDR;
                RAMOUT = LSRAMIN;
                RAMWRITE = 1; //Write to RAM
                end
            end
        
        if (Jtype) begin
            // PCOffset = {4'b0, IW[25:0] << 2};
            PCOffset = {4'b0, IW[25:0], 2'b0};
            $display("CTRL : got inside the j-type.");
            
            if (isLink) begin //If link instruction save PC into register 31
                WENREG = 1;
                Rd = 31;
                regData = PC;
                end
            end
        end
    
        
    
    //always @(*) begin 
        

        // if (FETCH) begin
        //     RAMADDR = PC; //Fetch instruction from PC
        //     RAMreadReq = 1;
        // end
    

//------------------------------------------------------------------------------------


        //else if (EXEC1) begin
            // RAMreadReq = 0;
            // WENREG = 0;
            // //IW = {RAMDATA[24:31], RAMDATA[16:23], RAMDATA[8:15], RAMDATA[0:7]}; //Endian conversion
            // IW = {RAMDATA[7:0], RAMDATA[15:8], RAMDATA[23:16], RAMDATA[31:24]};
            
            // // IW = RAMDATA;
            // if (Rtype) begin //R type instructions
            //     Rt = IW[20:16];
            //     Rs = IW[25:21];
            //     end
            // if (Itype) begin //I type instructions
            //     Rs = IW[25:21];
            //     if (isLoad) begin
            //         RAMreadReq = 1;
            //         RAMADDR = LSRAMADDR;
            //         end
            //     end
            // if (Jtype) begin //J type instructions
            //     Rs = 31;
            //     end
            //end

//------------------------------------------------------------------------------------

        //else if (EXEC2) begin
            // RAMreadReq = 0;
            // updatePC = 1;

            // if (Rtype) begin //R type instructions
            //     OP1 = RsDATA; //Set Rs
            //     OP2 = RtDATA; //Set Rt
            //     SHAMT = IW[10:6];
            //     Rd = IW[15:11];
            //     regData = RESULT; //Save result
            //     WENREG = 1;

            //     if (IW[5:0] == 6'b001000) begin 
            //         delay_halt_1 = 1;
            //         $display("CTRL : got inside the j-type.");
            //     end
            // end


            // if (Itype) begin  //Immediate Instructions
            //     $display("CTRL : got inside the i-type.");
            //     if (isArith) begin //Arithmetic immediate instructions
            //         OP1 = RsDATA;
            //         if(signReqd) begin
            //             OP2 = {16'b1111111111111111, IW[15:0]};
            //         end else begin
            //             OP2 = {16'b0, IW[15:0]};
            //         end
            //         SHAMT = 0;
            //         Rd = IW[15:11];
            //         regData = RESULT; //Save result
            //         WENREG = 1;
            //         end
            //     if (isBranch) begin //Branch instructions
            //         $display("CTRL : got inside the branch.");
            //         if (CONDITIONMET) begin 
            //             // PCOffset = {14'b0, IW[15:0] << 2}; //Calc PC offset
            //             // PCOffset = {14'b0, IW[15:0], 2'b0}; //Calc PC offset
            //             $display("CTRL : got inside the branch. ConditionMet: %h, EQ: %h", CONDITIONMET, EQ);
            //             delay_offset = {14'b0, IW[15:0], 2'b0};

            //             if (isLink) begin //If link instruction save PC into register 31
            //                 WENREG = 1;
            //                 Rd = 31;
            //                 regData = PC;
            //                 end
            //             end
            //         end
            //     if (isLoad) begin //Load instructions
            //         LSRSDATA = RsDATA; //Send RS data to LS
            //         LSRAMOUT = RAMDATA; //Send RAM data to LS
            //         Rd = IW[20:16]; //Save contents into register
            //         WENREG = 1;
            //         end
            //     if (isStore) begin //Store instructions
            //         LSRSDATA = RsDATA;
            //         LSRTDATA = RtDATA;
            //         RAMADDR = LSRAMADDR;
            //         RAMOUT = LSRAMIN;
            //         RAMWRITE = 1; //Write to RAM
            //         end
            //     end
            
            // if (Jtype) begin
            //     // PCOffset = {4'b0, IW[25:0] << 2};
            //     PCOffset = {4'b0, IW[25:0], 2'b0};
            //     $display("CTRL : got inside the j-type.");
                
            //     if (isLink) begin //If link instruction save PC into register 31
            //         WENREG = 1;
            //         Rd = 31;
            //         regData = PC;
            //         end
            //     end
            // end
        //end

    //Instantiate modules
    statemac cpuSM(
        //Outputs to state machine
        .clk(clock),
        .halt(HALT),
        .waitrequest(waitreqflag),
        .reset(RESET),

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
        .OPCODE(OPCODE),
        .FUNCCODE(FUNCCODE),
        .RT(RT)
    );

    ALU_comb cpuALU(
        //Outputs to ALU
        .clk(clock),
        .RESET(RESET),
        .a(OP1),
        .b(OP2),
        .shamt(SHAMT),
        .opcode(OPCODE),
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
        .WENREG(WENREG && !waitreqflag),
        .RESET(RESET),

        //Inputs from regFile
        .RsDATA(RsDATA),
        .RtDATA(RtDATA),
        .RdDATA(regData),
        .register_v0(regv0)
    );

    pc cpuPC(
        //Outputs to PC
        .clk(clock),
        .RESET(RESET),
        .HALT(HALT),
        .PCOffset(PCOffset),
        .updatePC(updatePC && !waitreqflag),

        //Inputs from PC
        .PC(PC)
    );

    loadandstore cpuLAS(
        //Outputs to LAS
        .cycle(EXEC1),
        .reg_s(LSRSDATA),
        .reg_t(LSRTDATA),
        .mem_read(LSRAMOUT),
        .instruction(LSIW),

        //Inputs from LAS
        .byteenable(byteEnable),
        .reg_write(RdDATA),
        .mem_write(LSRAMIN),
        //output logic reg_enable,
        //output logic[4:0] reg_write_index,
        .mem_address(LSRAMADDR)
    );
endmodule



/*

Implement conditions coming from the IR //
Support link instructions //
Determine how to counter a stall when writing to a register that is being operated on (save previous values?) //
Determine when a stall occurs - happens after a jump/branch //
Implement wait request //
Connect components together //
HALT causes PC output to mux to reset vector //
Determine when CPU halts //
PC needs reset flag to set it to 0 //
Regfile needs reset flag to set all registers to 0 and output register 1 //

------------TODO
Implement state machine



iverilog -Wall -g 2012 -s mips_cpu_bus_tb -o test/mips_cpu_bus_tb.vvp test/mips_cpu_bus_tb.v mips_cpu_bus.v statemac.v registerfile.v PC.v loadandstore.v IR.v ControlUnit.v ALU_comb.v 

*/