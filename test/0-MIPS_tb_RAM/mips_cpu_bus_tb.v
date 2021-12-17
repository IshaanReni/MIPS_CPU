module mips_cpu_bus_tb ();

timeunit 1ns / 10ps;

parameter RAM_INIT_FILE = "";
parameter TIMEOUT_CYCLES = 10000;

logic clk;
logic reset;
logic active;

logic[31:0] address;
logic       write_cpu;
logic       read_cpu;
logic       write_ram;
logic       read_ram;
logic[31:0] w_data;
logic[31:0] r_data;
logic[3:0] byte_en;
logic       wait_r_i;
logic       wait_r_o;

logic check_read;
logic check_write;


logic[31:0] reg_output;

RAM_8x_40000_avalon #(RAM_INIT_FILE) ramInst(
    .clk(clk),
    .write(write_ram),
    .read(read_ram),
    .address(address), 
    .write_data(w_data), 
    .read_data(r_data), 
    .byte_en(byte_en), 
    .in_waitreq(wait_r_i), 
    .waitreq(wait_r_o)
);
mips_cpu_bus cpuInst(
    .clk(clk), 
    .reset(reset), 
    .active(active), 
    .register_v0(reg_output),
    .address(address), 
    .write(write_cpu), 
    .read(read_cpu), 
    .waitrequest(wait_r_o), 
    .writedata(w_data), 
    .byteenable(byte_en), 
    .readdata(r_data)
);

    assign read_ram = read_cpu;
    assign  write_ram = write_cpu;

    // Generate clock
    initial begin
        clk=0;

        repeat (TIMEOUT_CYCLES) begin
            #5;
            clk = !clk;
            #5;
            clk = !clk;
        end

        $fatal(2, "Simulation did not finish within %d cycles.", TIMEOUT_CYCLES);
    end

    initial begin
        reset <= 0;

        // PC = 0h'BFC00000
        @(posedge clk);
        reset <= 1;

        @(posedge clk);
        reset <= 0;
        
        $display("TB : Reset is now low.");
        @(posedge clk);
        assert(active==1) else $display("TB : CPU did not set active=1 after reset.");

        while (active) begin
            @(posedge clk);
        end

        //tests have finished, cpu output "active" is l

        $display("TB : finished; running=0");
        $display("TB : INFO : register_v0=%h", reg_output);

        // $dumpfile("mips_cpu_bus_tb.vcd");
        // $dumpvars(0, mips_cpu_bus_tb);

        $finish;
        
    end

    //when waitrequest is high, read and write don't change. Stall in same state till we can read or write. So they should remain high.
    //no test of waitrequest yet

    always @(posedge clk) begin
        // Check read
        if(read_cpu && wait_r_i) begin 
            check_read <= 1;
        end
        if(check_read && wait_r_i) begin
            assert(read_cpu == 1) else $fatal(2, "TB : Read disabled during wait request");
        end
        if(check_read && !wait_r_i) begin
            check_read <= 0;
            assert(read_cpu == 1) else $fatal(2, "TB : Read not executed after wait request");
        end

        // Check write 
        if(write_cpu && wait_r_i) begin 
            check_write <= 1;
        end
        if(check_write && wait_r_i) begin
            assert(write_cpu == 1) else $fatal(2, "TB : Write disabled during wait request");
        end
        if(check_write && !wait_r_i) begin
            check_write <= 0;
            assert(write_cpu == 1) else $fatal(2, "TB : Write not executed after wait request");
        end
        if((read_cpu) && (write_cpu))begin
            $fatal(2,"TB : CPU tries to read from RAM while trying to write to it.");
        end
    end
    
    
    

endmodule