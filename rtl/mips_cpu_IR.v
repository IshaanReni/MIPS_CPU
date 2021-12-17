module IR(
    input logic[31:0] IW,

    output logic Rtype,
    output logic Itype,
    output logic Jtype,
    output logic isArith,
    output logic isBranch,
    output logic P,
    output logic N,
    output logic Z,
    output logic EQ,
    output logic NEQ,
    output logic link,
    output logic signReqd,
    output logic[5:0] OPCODE,
    output logic[5:0] FUNCCODE,
    output logic[4:0] RT
);

    logic[5:0] opcode;
    logic[4:0] rt;
    logic[5:0] funccode;

    always @(*) begin
        // $display("IR: INFO: IW: %h, OP: %b, Func: %h ", IW, opcode, funccode);

        Rtype = 0;
        Itype = 0;
        Jtype = 0;
        isArith = 0;
        isBranch = 0;
        P = 0;
        N = 0;
        Z = 0;
        EQ = 0;
        NEQ = 0;
        link = 0;
        opcode = IW[31:26];
        funccode = IW[5:0];
        rt = IW[20:16];


    
            //Is instruction R, I, or J type?
            if (opcode == 6'b000000) begin
                Rtype = 1;
                FUNCCODE = funccode;
            end
            else if ((opcode == 6'b000010) || (opcode == 6'b000011)) begin
                Jtype = 1;
                OPCODE = opcode;
            end
            else begin
                Itype = 1;
                if (((opcode == 6'b000001)&&(rt == 5'b00001)) || ((opcode == 6'b000001)&&(rt == 5'b10001))||((opcode == 6'b000001)&&(rt == 5'b00000))||((opcode == 6'b000001)&&(rt == 5'b10000))) begin
                    RT = rt;
                end
                else begin
                    OPCODE = opcode;
                    FUNCCODE = funccode;
                end
            end
        // $display("IR: INFO: IW: %h, OP: %b, Func: %h ", IW, opcode, funccode);
            //Are I Types Arithmetic or Branch?
            if ((opcode == 6'b001001) || (opcode == 6'b001100) || (opcode == 6'b001101) || (opcode == 6'b001110)) begin
                isArith = 1;
            end

            if ((opcode == 6'b000100) || (opcode == 6'b000001) || (opcode == 6'b000111) || (opcode == 6'b000110) || (opcode == 6'b000101)) begin
                isBranch = 1;
            end

            //For Branch Instr. does ALU calculation produce Positive, Negative, Equal or Zero value?
            if (opcode == 6'b000100) begin //BEQ
                EQ = 1;
            end
            if (opcode == 6'b000111) begin //BGTZ
                P = 1; //P means positive and non-zero btw
            end
            if (((opcode == 6'b000001) && (rt == 5'b10001)) || ((opcode == 6'b000001) && (rt == 5'b00001))) begin //BGEZ and BGEZAL
                P = 1;
                Z = 1;
            end
            if (opcode == 6'b000110) begin //BLEZ
                N = 1;
                Z = 1;
            end
            if (((opcode == 6'b000001) && (rt == 5'b00000)) || ((opcode == 6'b000001) && (rt == 5'b10000))) begin //BLTZ and BLTZAL
                N = 1;
            end
            if (opcode == 6'b000101) begin //BNE
                NEQ = 1;
            end

            //If link?
            if ((opcode == 6'b000011) || ((Rtype == 1) && (funccode == 6'b001001)) || ((opcode == 6'b000001) && (rt == 5'b10001)) || ((opcode == 6'b000001) && (rt == 5'b10000))) begin
                link = 1;
            end

            if((opcode == 6'b001001) || (opcode == 6'b001010) || (opcode == 6'b001011)) begin
                signReqd = 1;
            end


    end
    
endmodule