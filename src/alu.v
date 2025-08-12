module alu(
    input [31:0] input_a,
    input [31:0] input_b,
    input [3:0] alu_op,
    output reg [31:0] alu_result,
    output zero
);
    always @(*) begin
        case (alu_op)
            4'b0000: alu_result = input_a + input_b;        // ADD
            4'b0001: alu_result = input_a - input_b;        // SUB
            4'b0010: alu_result = input_a & input_b;        // AND
            4'b0011: alu_result = input_a | input_b;        // OR
            4'b0100: alu_result = input_a ^ input_b;        // XOR
            4'b0101: alu_result = input_a << input_b[4:0];  // SLL
            4'b0110: alu_result = input_a >> input_b[4:0];  // SRL
            4'b0111: alu_result = ($signed(input_a) < $signed(input_b)) ? 32'd1 : 32'd0; // SLT
            4'b1000: alu_result = (input_a < input_b) ? 32'd1 : 32'd0; // SLTU
            default: alu_result = 32'd0;
        endcase
    end
    assign zero = (alu_result == 0);
endmodule

