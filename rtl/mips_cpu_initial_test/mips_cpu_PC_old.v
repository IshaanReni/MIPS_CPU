module pc ( //output pc counter, inputs pc offset, update_pc 
    input logic clk,
    input logic reset,
    input logic HALT,
    //input logic HALT,
    input logic[31:0] PCOffset, //
    input logic updatePC, //
    input logic[31:0] reg_value, 
    input logic jump_r, 
    output logic[31:0] PC //
);  
    logic[31:0] PCdelay = 0;
    //PC = 32'hBFC00000;
    always @(posedge clk)begin
        if (reset) begin
            PC <= 32'hBFC00000;
        end 
        else if(HALT) begin
            PC <= 32'h00000000;
        end
        else begin
            if(jump_r && updatePC) begin
                PC <= reg_value * 4;
            end
            else if(updatePC) begin
                // PCdelay <= PCOffset;
                // PC <= PC + 4 + PCdelay;
                $display("PC: About to update the PC. It is currently:%h", PC);
                PC <= PC + 4;
                // PC <= PC + 4 + PCOffset;
            end 
        end
    end

endmodule