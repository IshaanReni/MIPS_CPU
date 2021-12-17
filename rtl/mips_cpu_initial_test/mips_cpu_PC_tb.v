module mips_cpu_PC_tb();

    logic in_clk;
    logic in_RESET;
    logic in_HALT;
    logic[31:0] in_PC_Offset;
    logic in_updatePC;
    logic[31:0] in_reg_jump_value;
    logic in_jump_r;
    logic in_jump_const;
    logic[31:0] out_PC;
    logic active;

    pc alwaysright( 
        .clk(in_clk),
        .RESET(in_RESET),
        .HALT(in_HALT),
        .PCOffset(in_PC_Offset), 
        .updatePC(in_updatePC), 
        .reg_jump_value(in_reg_jump_value), 
        .jump_r(in_jump_r), 
        .jump_const(in_jump_const), 
        .PC(out_PC) 
    );   

    initial begin
        repeat(100) begin
            #5;
            in_clk = 1;
            #5;
            in_clk = 0;
        end

        active = 0;
    end 

    initial begin
        active = 1;

        @(posedge in_clk);
        in_RESET = 1;

        @(posedge in_clk);
        in_RESET = 0;

        repeat(3) begin
            @(posedge in_clk);
            $display("PC: %h", out_PC);
            in_updatePC = 1;
            in_jump_r = 0;
            in_jump_const = 0;
            in_HALT = 0;
            in_RESET = 0;
            #1;
        end

        @(posedge in_clk);
        in_RESET = 1; 

        @(posedge in_clk);
        $display("PC reset");
        $display("PC: %h", out_PC);
        assert(out_PC == 32'hbfc00000) else $display("Reset malfunction");

        in_RESET = 0;
        @(posedge in_clk);

        repeat(4) begin
            @(posedge in_clk);
            $display("PC: %h", out_PC);
            in_updatePC = 1;
            in_jump_r = 0;
            in_jump_const = 0;
            in_HALT = 0;
            in_RESET = 0;
            #1;
        end

        $display("PC: %h", out_PC);
        in_jump_r = 1;
        in_reg_jump_value = 32'h00dd1234;

        @(posedge in_clk);
        $display("JR");

        repeat(4) begin
            @(posedge in_clk);
            $display("PC: %h", out_PC);
            in_updatePC = 1;
            in_jump_r = 0;
            in_jump_const = 0;
            in_HALT = 0;
            in_RESET = 0;
            #1;
        end

        $display("PC: %h", out_PC);
        in_jump_const = 1;
        in_PC_Offset = 32'h00088004;

        @(posedge in_clk);
        $display("Branch");

        repeat(4) begin
            @(posedge in_clk);
            $display("PC: %h", out_PC);
            in_updatePC = 1;
            in_jump_r = 0;
            in_jump_const = 0;
            in_HALT = 0;
            in_RESET = 0;
            #1;
        end

        @(posedge in_clk);
        in_HALT = 1;
        
        @(posedge in_clk);
        $display("PC halted");
        $display("PC: %h", out_PC);
        assert(out_PC == 32'h00000000) else $display("Halt malfunction");

    end

endmodule





