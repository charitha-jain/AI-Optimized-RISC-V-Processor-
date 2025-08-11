module alu(
    input  [127:0] input_a,
    input  [127:0] input_b,
    input  [3:0]   alu_op,
    output reg [127:0] alu_result,
    output zero
);
    integer i;
    reg [31:0] a_lane, b_lane;
    reg [31:0] res_lane;
    reg [127:0] result_next;

    always @(*) begin
        result_next = 128'b0;
        for (i = 0; i < 4; i = i + 1) begin
            a_lane = input_a[32*i +: 32];
            b_lane = input_b[32*i +: 32];
            case (alu_op)
                4'b0000: res_lane = a_lane + b_lane;          // ADD
                4'b0001: res_lane = a_lane - b_lane;          // SUB
                4'b0010: res_lane = a_lane & b_lane;          // AND
                4'b0011: res_lane = a_lane | b_lane;          // OR
                4'b0100: res_lane = a_lane ^ b_lane;          // XOR
                4'b0101: res_lane = a_lane << b_lane[4:0];    // SLL
                4'b0110: res_lane = a_lane >> b_lane[4:0];    // SRL
                default: res_lane = 32'b0;
            endcase
            result_next[32*i +: 32] = res_lane;
        end
    end

    always @(*) begin
        alu_result = result_next;
    end
    assign zero = (alu_result[31:0] == 32'b0);

endmodule

