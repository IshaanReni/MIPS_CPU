module RAM_8x_40000_avalon (
    input   logic       clk,
    input   logic       write,
    input   logic       read,
    input   logic[31:0] address,
    input   logic[31:0] write_data,
    input   logic[3:0]  byte_en,
    output  logic[31:0] read_data,  

    input   logic       in_waitreq,
    output  logic       waitreq
);

parameter RAM_INIT_FILE = "";

logic[31:0] mapped_address;

// 8x8MB 
reg [7:0] memory [39999:0];

initial begin
    integer i;
    integer j;
    // sets all memory locations to zero 
    for (i=0; i<40000; i++) begin
        memory[i]=0;
    end
    // loads into memory from file RAM_INIT_FILE 
    // starts at location 0h'BFC00000, AKA 20000 in our mapped RAM (also location after reset)
    if (RAM_INIT_FILE != "") begin
        $display("RAM : INIT : Loading RAM contents from %s", RAM_INIT_FILE);
        $readmemh(RAM_INIT_FILE, memory,20000,39999); 
    end  
    
    for(j=20000; j < 20030; j++)begin
        $display("RAM : INIT : RAM contents at %h are: %h", j, memory[j]);
    end
    
end 

// waitrequest is passed on as ouput going to FSM and LS
assign waitreq = in_waitreq;

// we only want to output or write data when we are not in a wait_req state, should it be clk instead and check waitreq not high
always @(posedge clk && !waitreq)begin
    
    // endianess is converted in the load store!
    if(read) begin
        if(address < 20000) begin
            read_data = {memory[address+3], memory[address+2], memory[address+1], memory[address]};
        end
        // maps addresses higher than 0xBFC00000 to addresses starting at mapped location 20,000
        else if((address >= 3217031168) && (address < 3217051168)) begin //0xBFC00000  to 0xBFC04E20
            read_data = {memory[address-3217011168+3], memory[address-3217011168+2], 
                        memory[address-3217011168+1], memory[address-3217011168]};
            
            // if(address < 3217031218) begin
            //     $display("RAM: reading data at location: %h  data: %h", address, read_data);
            // end 
        end
        else begin
            //$display("entering read_data else: read_data is: %h", read_data);
            read_data = 32'h00000000;
        end
    end

    // data is already big endian coming in
    if(write) begin
        if(address < 20000) begin
            mapped_address = address;
        end
        else if((address >= 3217031168) && (address < 3217051168)) begin
            mapped_address = address - 3217031168;
        end

        if(byte_en[0] == 1)begin
            memory[mapped_address] = write_data[7:0];
        end
        if(byte_en[1] == 1)begin
            memory[mapped_address+1] = write_data[15:8];
        end
        if(byte_en[2] == 1)begin
            memory[mapped_address+2] = write_data[23:16];
        end
        if(byte_en[3] == 1)begin
            memory[mapped_address+3] = write_data[31:24];
        end
    end
end

endmodule