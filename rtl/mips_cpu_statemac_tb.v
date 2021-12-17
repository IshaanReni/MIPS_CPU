module mips_cpu_statemac_tb ();
    logic halt;
    logic waitrequest;
    logic reset;
    logic clk;
    logic active;
    logic f;
    logic e1;
    logic e2;
    
    statemac fsm(
        .halt(halt),
        .waitrequest(waitrequest),
        .reset(reset),
        .clk(clk),
        .active(active),
        .f(f),
        .e1(e1),
        .e2(e2)
    );
    
    initial begin
        $dumpfile("mips_cpu_statemac_tb.vcd");
        $dumpvars(0, mips_cpu_statemac_tb);

        halt = 1;
        active = 0;
        waitrequest = 0;
        

        $display("Current State: f=%b, e1=%b, e2=%b", f, e1, e2);

        repeat(200) begin
            clk <= 0;
            #1;
            clk <= 1;
            #1;
        end

        #1
        halt = 0;
        #1;
        assert(active == 1); 

        #1;
        halt = 1;
        #1;
        assert(active == 0);

        
    end


endmodule