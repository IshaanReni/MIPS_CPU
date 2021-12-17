module statemac(
    input logic halt,
    input logic waitrequest,
    input logic reset,
    input logic clk,
    output logic active,
    output logic f,
    output logic e1,
    output logic e2
);

parameter halt_state = 3'b000, fetch = 3'b001, exec1 = 3'b011, exec2 = 3'b101, stall_f = 3'b010, stall_e1 = 3'b100, stall_e2 = 3'b110;
reg [2:0] state = fetch;
/*
 if ((reset) && (state != halt_state)) begin
        state <= fetch;
    end
    else begin*/
always @(posedge clk) begin
        f = 0;
        e1 = 0;
        e2 = 0;
   
        case (state)

        fetch: begin
            $display("FSM: FETCH");
            active = 1;
            if (waitrequest) begin
                state <= stall_f;
            end
            else if (halt) begin
                state <= halt_state;
            end
            else if (reset) begin
                state <= fetch;
            end
            else begin
                state <= exec1;
            end
            f = 1;
        end

        exec1: begin
            $display("FSM: EXEC1");
            active = 1;
            if (waitrequest) begin
                state <= stall_e1;
            end
            else if (reset) begin
                state <= fetch;
            end
            else begin
                state <= exec2;
            end
            e1 = 1;
        end

        exec2: begin
            $display("FSM: EXEC2");
            active = 1;
            if (waitrequest) begin
                state <= stall_e2;
            end
            else begin
                state <= fetch;
            end
            e2 = 1; 
        end

        halt_state: begin
            $display("FSM: HALT");
            active = 0;
            if (reset) begin
                state <= fetch;
            end
            else begin
                state <= halt_state;
            end
        end

        stall_f: begin
            active = 1;
            if (!waitrequest) begin
                state <= fetch;
            end
            else begin
                state <= stall_f;
            end
        end

        stall_e1: begin
            active = 1;
            if (!waitrequest) begin
                state <= exec1;
            end
            else begin
                state <= stall_e1;
            end
        end

        stall_e2: begin
            active = 1;
            if (!waitrequest) begin
                state <= exec2;
            end
            else begin
                state <= stall_e2;
            end
        end


        endcase

    end


endmodule
