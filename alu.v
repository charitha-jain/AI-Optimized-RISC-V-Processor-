module alu(
    input [31:0] input_a,
    input [31:0] input_b,
    input [3:0] alu_op,
    output reg [31:0] alu_result,
    output zero
);
    always @(*) begin
        case (alu_op)
            4'b0000: alu_result = input_a + input_b;
            4'b0001: alu_result = input_a - input_b;
            4'b0010: alu_result = input_a & input_b;
            4'b0011: alu_result = input_a | input_b;
            4'b0100: alu_result = input_a ^ input_b;
            4'b0101: alu_result = input_a << input_b[4:0];
            4'b0110: alu_result = input_a >> input_b[4:0];
            default: alu_result = 0;
        endcase
    end
    assign zero = (alu_result == 0);
endmodule

