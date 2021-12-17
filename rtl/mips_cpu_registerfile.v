module registerfile (
	input logic clk,
	input logic WENREG,
	input logic reset,
	input logic [4:0] Rs,
	input logic [4:0] Rt,
    	input logic [4:0] Rd,
	input logic [31:0] RdDATA,
	output logic [31:0] RsDATA,
	output logic [31:0] RtDATA,
	output logic [31:0] register_v0
	);

logic[31:0] regFile [0:31];

always_ff @(posedge clk or posedge reset) begin
	RsDATA <= regFile[int'(Rs)];
	RtDATA <= regFile[int'(Rt)];
	register_v0 <= regFile[2];

	if (reset) begin
		for (int i=0; i < 32; i++) begin
			regFile[i] <= 0;
		end
	end 
	else if (WENREG) begin
		regFile[int'(Rd)] <= RdDATA;
	end
end


endmodule
