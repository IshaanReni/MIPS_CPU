module mips_cpu_bus(
    /* Standard signals */
    input logic clk, //
    input logic reset, //
    output logic active, 
    output logic[31:0] register_v0,

    /* Avalon memory mapped bus controller (master) */
    output logic[31:0] address, //
    output logic write, //
    output logic read, //
    input logic waitrequest, //
    output logic[31:0] writedata, //
    output logic[3:0] byteenable, //
    input logic[31:0] readdata //
);


    ControlUnit cpuCU(
        //Outputs to Control Unit
        .clock(clk),
        .reset(reset),
        .waitrequest(waitrequest),
        .RAMDATA(readdata),

        //Inputs from Control Unit
        .RAMADDR(address),
        .RAMWRITE(write),
        .RAMreadReq(read),
        .ACTIVE(active),
        .byteEnable(byteenable),
        .LSRAMIN(writedata),
        .regv0(register_v0)
    );

    logic is_active;
    logic is_reset;
    logic[31:0] instructions;
    logic[31:0] read_mem;

    always @(posedge is_reset)begin
        $display("CPU : INFO   : Resetting.");
        // $display("CPU : INFO   : Fetching instruction=%h", instructions);
    end
    always @(posedge clk) begin
        is_active = active;
        is_reset = reset;
        instructions = address;
        read_mem = readdata;

        if(read)begin
           //$display("CPU : INFO   : Read instruction=%h", instructions);
        end

    end
    always @(negedge is_active)begin
        $display("CPU : OUT   : %d", register_v0);
    end

    

endmodule