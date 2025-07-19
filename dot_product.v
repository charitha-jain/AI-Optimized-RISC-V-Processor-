module dot_product(
    input [127:0] vec_a, vec_b,
    output reg [31:0] dot_result
);
    integer i;
    reg [63:0] acc;  // accumulator with enough width

    always @(*) begin
        acc = 0;
        for (i = 0; i < 4; i = i + 1) begin  // assuming 4 elements of 32 bits
            acc = acc + vec_a[32*i +: 32] * vec_b[32*i +: 32];
        end
        dot_result = acc[31:0];
    end
endmodule
