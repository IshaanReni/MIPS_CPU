module mips_cpu_bus_tb();
    logic clk, reset, active, write, read, waitrequest;
    logic [31:0] register_v0, address, writedata, readdata;
    logic [3:0] byteenable;
    
    ControlUnit cpuCU(
        .clock(clk),
        .RESET(reset),
        .waitrequest(waitrequest),
        .RAMDATA(readdata),
        .RAMADDR(address),
        .RAMWRITE(write),
        .RAMreadReq(read),
        .byteEnable(byteenable),
        .LSRAMIN(writedata)
    );

    initial begin
        repeat(200) begin
            clk = 0;
            #1;
            clk = 1;
            #1;
        end
    end

    initial begin
        $dumpfile("mips_cpu_bus_tb.vcd");
        $dumpvars(0, mips_cpu_bus_tb);
        @(posedge read);
        readdata = 32'b00100100000000010000000000001111;//ADDIU R0,R1,0x000F. Expect R1 = 0x000F now.
    end
    /*Instructions to add: JR, ADDU, ADDIU, LW, SW

    initial begin
        $dumpfile("mips_cpu_bus_tb.vcd");
        $dumpvars(0, mips_cpu_bus_tb);
        reset = 0;
        active = 1;
        read = 0;
        write = 0;
        waitrequest = 0;
        register_v0 = 32'b0;
        address = 32'b0;
        writedata = 32'b0;
        readdata = 32'b0;
        #1;
        read = 1;
        readdata = 32'b00100100000000010000000000001111;//ADDIU R0,R1,0x000F. Expect R1 = 0x000F now.
        #1;
        read = 0;
        #2;
        read = 1;
        readdata = 32'b;//SW R1, 0x001F(R0)
        #1;
        read = 0;
        #2;
        read = 1;
        readdata = 32'b;//ADDU R
        #1;
        read = 0;
        #2;
        read = 1;
        readdata = 32'b;//JR . Jump to register_v0.
        #1;
        read = 0;
        #2;
        read = 1;
        readdata = 32'b;//LW R0, 0x001F(R0). Should load the value 0x000F into R1
        #1;
        read = 0;
        #2;
        read = 1;
        readdata = 32'b;//HALT
        #1;
        read = 0;
        #2;
    end
*/
endmodule
