module dot_product(
    input [127:0] vec_a, vec_b,
    output reg [127:0] dot_result
);
    integer i;
    reg signed [63:0] acc;
    reg signed [31:0] a_lane, b_lane;
    always @(*) begin
        acc = 0;
        for (i = 0; i < 4; i = i + 1) begin
            a_lane = vec_a[32*i +: 32];
            b_lane = vec_b[32*i +: 32];
            acc = acc + (a_lane * b_lane);
        end
        dot_result = {96'b0, acc[31:0]};
    end
endmodule

