module dot_product(
    input [31:0] vec_a,
    input [31:0] vec_b,
    output reg [31:0] dot_result
);
    always @(*) begin
        dot_result = vec_a * vec_b;
    end
endmodule

