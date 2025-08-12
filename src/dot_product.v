module dot_product (
    input  wire [31:0] vec_a,
    input  wire [31:0] vec_b,
    output reg  [31:0] dot_result
);
    reg [63:0] product;
    always @(*) begin
        product = $signed(vec_a) * $signed(vec_b); // signed multiply; change if unsigned
        dot_result = product[31:0]; // truncate to 32 bits
    end
endmodule

