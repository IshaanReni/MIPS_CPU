module mips_cpu_registerfile_tb();

    logic in_clk;
    logic in_WENREG;
    logic in_RESET;
    logic[4:0] in_Rs;
    logic[4:0] in_Rt;
    logic[4:0] in_Rd;
	logic[31:0] in_write_Rd;
	logic[31:0] out_read_Rs;
	logic[31:0] out_read_Rt;
	logic[31:0] out_read_v0;

    registerfile reg_file(
	    .clk(in_clk),
	    .WENREG(in_WENREG),
	    .RESET(in_RESET),
	    .Rs(in_Rs),
	    .Rt(in_Rt),
        .Rd(in_Rd),
	    .RdDATA(in_write_Rd),
	    .RsDATA(out_read_Rs),
	    .RtDATA(out_read_Rt),
	    .register_v0(out_read_v0)
    );


    initial begin
        repeat (100) begin
            #5;
            in_clk = 1;
            #5;
            in_clk = 0;
        end
    end

    initial begin
        @(posedge in_clk);
        in_WENREG = 1;
        in_RESET = 0;
        in_Rd = 2;
        in_write_Rd = 17 + 32;

        #20
        
        @(posedge in_clk);
        in_WENREG = 0;
        in_RESET = 0;
        in_Rs = 2;
        in_Rt = 3;

        #15

        @(posedge in_clk);
        assert(out_read_Rs == 49) else $display("Test 1 Rs read incorrect: %d", out_read_Rs);
        assert(out_read_Rt == 0) else $display("Test 1 Rt read incorrect: %d", out_read_Rt);
        assert(out_read_v0 == 49) else $display("Test 1 v0 read incorrect: %d", out_read_v0);
        
        #10;
        @(posedge in_clk);
        in_WENREG = 1;
        in_Rd = 3;
        in_Rs = 2;
        in_write_Rd = 195*195;

        #10;

        assert(out_read_Rs == 49) else $display("Test 2 Rs read incorrect: %d", out_read_Rs);
        assert(out_read_v0 == 49) else $display("Test 1 v0 read incorrect: %d", out_read_v0);

        #10;
        @(posedge in_clk);
        in_WENREG = 0;
        in_Rs = 3;
        in_Rt = 2;

        #10;

        assert(out_read_Rs == 38025) else $display("Test 3 Rs read incorrect: %d", out_read_Rs);
        assert(out_read_Rt == 49) else $display("Test 3 Rt read incorrect: %d", out_read_Rt);
        assert(out_read_v0 == 49) else $display("Test 1 v0 read incorrect: %d", out_read_v0);

    end

endmodule



