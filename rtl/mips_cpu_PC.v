module pc ( //output pc counter, inputs pc offset, update_pc 
    input logic clk,
    input logic reset,
    input logic halt,
    input logic[31:0] PCOffset, 
    input logic updatePC, 
    input logic[31:0] reg_jump_value, 
    input logic jump_r, 
    input logic jump_const, 
    output logic[31:0] PC 
);  
    logic[31:0] PC_offset_delayed;
    logic[31:0] reg_value_delayed;
    logic r_delay, o_delay;

    always @(posedge clk)begin
        if (reset) begin
            PC <= 32'hBFC00000;
            r_delay = 0;
            o_delay = 0;
        end
        else if(halt) begin
            PC <= 32'h00000000;
        end
        else begin
            $display("PC: not reset or halt, updatePC: %b", updatePC);

            if (updatePC) begin
                $display("PC: current value before increment: %h", PC);

                // if(jump_r) begin
                //     $display("PC: In jump_r");
                //     PC <= reg_jump_value * 4;
                // end
                // else if(jump_const) begin
                //     $display("PC: In jump_const");
                //     PC <= PC + PCOffset;
                // end
                // else begin
                //     $display("PC: normal increment");
                //     PC <= PC + 4;
                // end

                if(r_delay) begin
                    $display("PC: In r_delay: %h", reg_value_delayed);
                    PC <= reg_value_delayed * 4;
                    r_delay <= 0;
                end
                else if(o_delay) begin
                    $display("PC: In o_delay");
                    PC <= PC + PC_offset_delayed;
                    o_delay <= 0;
                end
                else if(jump_r) begin
                    $display("PC: In jump_r");
                    reg_value_delayed <= reg_jump_value;
                    r_delay <= 1;
                    PC <= PC + 4;
                end
                else if(jump_const) begin
                    $display("PC: In jump_constant");
                    PC_offset_delayed <= PCOffset;
                    o_delay <= 1;
                    PC <= PC + 4;
                end
                else begin
                    PC <= PC + 4;
                end 

            end
        end
    end

endmodule