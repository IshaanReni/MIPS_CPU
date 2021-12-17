module extend8(
    input logic[7:0] a, 
    output logic[31:0] a_extended, 
    output logic[31:0] l_extended
);

    assign a_extended = {24'hFFFFFF, a};
    assign l_extended = {24'h000000, a};

endmodule

module extend16(
    input logic[15:0] a, 
    output logic[31:0] a_extended, 
    output logic[31:0] l_extended
);

    assign a_extended = {16'hFFFF, a};
    assign l_extended = {16'h0000, a};

endmodule

module loadandstore(
    input logic cycle, 
    input logic fetch, 
    input logic[31:0] reg_s, 
    input logic[31:0] reg_t, 
    input logic[31:0] mem_read, 
    input logic[31:0] instruction,
    output logic[3:0] byteenable, 
    output logic[31:0] reg_write, 
    output logic[31:0] mem_write, 
    output logic reg_enable,
    output logic[4:0] reg_write_index,
    output logic[31:0] mem_address, 
    output logic write_enable, 
    output logic read_enable
);
    logic u;
    logic[2:0] opcode1;
    logic[2:0] opcode2;
    logic[7:0] mem_byte;
    logic[15:0] mem_half;
    logic[15:0] offset;
    logic[31:0] tmp_address;

    logic[31:0] l_extended_byte, a_extended_byte, l_extended_half, a_extended_half;

    assign opcode1 = instruction[31:29];
    assign opcode2 = instruction[28:26];
    assign offset = instruction[15:0];
 
    always@(*) begin  
        write_enable = 0;
        read_enable = 0;
        byteenable = 4'b0000;

        // if load instruction, else store instruction
        if (opcode1 == 3'b100) begin
            $display("Mem read: %h", mem_read);

            // when exec 1, else exec 2
            if (cycle == 1) begin
                // mem_address = source register + offset
                tmp_address = reg_s + {16'h0000, offset};
                mem_address = {tmp_address[15:2], 2'b00};
                byteenable = 4'b1111;
                read_enable = 1;
                $display("Mem address access: %h", mem_address);

            end
            else begin
                // load mem_write to destination register 
                reg_enable = 1;
                reg_write_index = instruction[20:16];
                read_enable = 0;
                byteenable = 4'b0000;

                // sort word, half, and byte 
                case (opcode2) 
                    3'b011 : reg_write = {mem_read[7:0], mem_read[15:8], mem_read[23:16], mem_read[31:24]}; // LW
                    3'b000 : begin 
                        byteenable = 4'b1111;

                        case (mem_address[1:0])  //LB
                            2'b11 : mem_byte = mem_read[7:0];
                            2'b10 : mem_byte = mem_read[15:8];
                            2'b01 : mem_byte = mem_read[23:16];
                            2'b00 : mem_byte = mem_read[31:24];
                            default : mem_byte = 8'h00;
                        endcase

                        if (mem_byte[7] == 1) begin
                            reg_write = a_extended_byte;
                        end
                        else begin
                            reg_write = l_extended_byte;
                        end

                    end
                    3'b100 : begin
                        byteenable = 4'b1111;

                        case (mem_address[1:0])   //LBU
                            2'b11 : mem_byte = mem_read[7:0];
                            2'b10 : mem_byte = mem_read[15:8];
                            2'b01 : mem_byte = mem_read[23:16];
                            2'b00 : mem_byte = mem_read[31:24];
                            default : mem_byte = 8'h00;
                        endcase

                        reg_write = l_extended_byte;
                    end
                    3'b001 : begin
                        byteenable = 4'b1111;

                        case (mem_address[1:0])  // LH
                            2'b10 : mem_half = {mem_read[7:0], mem_read[15:8]};
                            2'b00 : mem_half = {mem_read[23:16], mem_read[31:24]};
                            default : mem_half = 16'h0000;
                        endcase
                        // reg_write = {16'h0000, mem_read[7:0], mem_read[15:8]};

                        if (mem_half[15] == 1) begin
                            reg_write = a_extended_half;
                        end
                        else begin
                            reg_write = l_extended_half;
                        end
                    end
                    3'b101 : begin
                        byteenable = 4'b1111;

                        case (mem_address[1:0])  // LHU
                            2'b10 : mem_half = {mem_read[7:0], mem_read[15:8]};
                            2'b00 : mem_half = {mem_read[23:16], mem_read[31:24]};
                            default : mem_half = 16'h0000;
                        endcase

                        reg_write = l_extended_half;
                    end
                    3'b110 : begin // LWL
                        byteenable = 4'b1111;

                        case (mem_address[1:0]) 
                            2'b00 : reg_write = {mem_read[7:0], mem_read[15:8], mem_read[23:16], mem_read[31:24]};
                            2'b01 : reg_write = {mem_read[15:8], mem_read[23:16], mem_read[31:24], reg_t[7:0]};
                            2'b10 : reg_write = {mem_read[23:16], mem_read[31:24], reg_t[15:0]};
                            2'b11 : reg_write = {mem_read[31:24], reg_t[23:0]};
                            default : reg_write = 32'h00000000;
                        endcase
                    end
                    3'b111 : begin // LWR
                        byteenable = 4'b1111;

                        case (mem_address[1:0]) 
                            2'b00 : reg_write = {reg_t[31:8], mem_read[7:0]};
                            2'b01 : reg_write = {reg_t[31:16], mem_read[7:0], mem_read[15:8]};
                            2'b10 : reg_write = {reg_t[31:24], mem_read[7:0], mem_read[15:8], mem_read[23:16]};
                            2'b11 : reg_write = {mem_read[7:0], mem_read[15:8], mem_read[23:16], mem_read[31:24]};
                            default : reg_write = 32'h00000000;
                        endcase
                    end
                    default : reg_write = 32'h00000000;
                endcase
            end
        end
        else if ((opcode1 == 3'b001) && (opcode2 == 3'b111)) begin
            // Load upper immediate, load last 16 bits of instruction into register in exec 2
            if ((cycle != 1) && (fetch != 1)) begin
                reg_write = {offset, 16'h0000};
                reg_write_index = instruction[20:16];
                reg_enable = 1;
            end
        end 
        else if (opcode1 == 3'b101) begin            
            // Store word, half, and byte in exec 2
            if ((cycle != 1) && (fetch != 1)) begin
                write_enable = 1;
                mem_address = reg_s + {16'h0000, offset};
                mem_write = {reg_t[7:0], reg_t[15:8], reg_t[23:16], reg_t[31:24]};

                // Sort word, half, and byte 
                case (opcode2) 
                    3'b000 : begin 
                        case (mem_address[1:0])  // SB
                            2'b00 : byteenable = 4'b0001;
                            2'b01 : byteenable = 4'b0010;
                            2'b10 : byteenable = 4'b0100;
                            2'b11 : byteenable = 4'b1000;
                            default : byteenable = 4'b0000;
                        endcase
                    end 
                    3'b001 : begin 
                        case (mem_address[1:0])  // SH
                            2'b00 : byteenable = 4'b0011;
                            2'b10 : byteenable = 4'b1100;
                            default : byteenable = 4'b0000;
                        endcase
                    end
                    3'b011 : byteenable = 4'b1111; // SW
                    default : byteenable = 4'b0000; 
                endcase
            end
        end
    end 

    extend8 b(.a(mem_byte), .a_extended(a_extended_byte), .l_extended(l_extended_byte));

    extend16 h(.a(mem_half), .a_extended(a_extended_half), .l_extended(l_extended_half));
    
endmodule




