module alu(
    input  [127:0] input_a,
    input  [127:0] input_b,
    input  [3:0]   alu_op,
    output reg [127:0] alu_result,
    output zero
);
    reg [31:0] res_lane0, res_lane1, res_lane2, res_lane3;

    always @(*) begin
        case (alu_op)
            4'b0000: res_lane0 = input_a[31:0]    + input_b[31:0];
            4'b0001: res_lane0 = input_a[31:0]    - input_b[31:0];
            4'b0010: res_lane0 = input_a[31:0]    & input_b[31:0];
            4'b0011: res_lane0 = input_a[31:0]    | input_b[31:0];
            4'b0100: res_lane0 = input_a[31:0]    ^ input_b[31:0];
            4'b0101: res_lane0 = input_a[31:0] << input_b[4:0];
            4'b0110: res_lane0 = input_a[31:0] >> input_b[4:0];
            default: res_lane0 = 32'b0;
        endcase

        case (alu_op)
            4'b0000: res_lane1 = input_a[63:32]   + input_b[63:32];
            4'b0001: res_lane1 = input_a[63:32]   - input_b[63:32];
            4'b0010: res_lane1 = input_a[63:32]   & input_b[63:32];
            4'b0011: res_lane1 = input_a[63:32]   | input_b[63:32];
            4'b0100: res_lane1 = input_a[63:32]   ^ input_b[63:32];
            4'b0101: res_lane1 = input_a[63:32] << input_b[36:32];
            4'b0110: res_lane1 = input_a[63:32] >> input_b[36:32];
            default: res_lane1 = 32'b0;
        endcase

        case (alu_op)
            4'b0000: res_lane2 = input_a[95:64]   + input_b[95:64];
            4'b0001: res_lane2 = input_a[95:64]   - input_b[95:64];
            4'b0010: res_lane2 = input_a[95:64]   & input_b[95:64];
            4'b0011: res_lane2 = input_a[95:64]   | input_b[95:64];
            4'b0100: res_lane2 = input_a[95:64]   ^ input_b[95:64];
            4'b0101: res_lane2 = input_a[95:64] << input_b[68:64];
            4'b0110: res_lane2 = input_a[95:64] >> input_b[68:64];
            default: res_lane2 = 32'b0;
        endcase

        case (alu_op)
            4'b0000: res_lane3 = input_a[127:96]  + input_b[127:96];
            4'b0001: res_lane3 = input_a[127:96]  - input_b[127:96];
            4'b0010: res_lane3 = input_a[127:96]  & input_b[127:96];
            4'b0011: res_lane3 = input_a[127:96]  | input_b[127:96];
            4'b0100: res_lane3 = input_a[127:96]  ^ input_b[127:96];
            4'b0101: res_lane3 = input_a[127:96] << input_b[100:96];
            4'b0110: res_lane3 = input_a[127:96] >> input_b[100:96];
	 endcase

	alu_result = {res_lane3, res_lane2, res_lane1, res_lane0};
    end

    assign zero = (alu_result[31:0] == 32'b0);

endmodule	
