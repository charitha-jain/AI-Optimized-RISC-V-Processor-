module matrix_multiplier(
    input clk,
    input start,
    input [255:0] matrix_a,
    input [255:0] matrix_b,
    output reg done,
    output reg [255:0] result
);
    reg signed [63:0] prod_lane0, prod_lane1, prod_lane2, prod_lane3;
    reg [127:0] low_products;

    always @(posedge clk) begin
        if (start) begin
            prod_lane0 = $signed(matrix_a[31:0])    * $signed(matrix_b[31:0]);
            low_products[31:0] = prod_lane0[31:0];

            prod_lane1 = $signed(matrix_a[63:32])   * $signed(matrix_b[63:32]);
            low_products[63:32] = prod_lane1[31:0];

            prod_lane2 = $signed(matrix_a[95:64])   * $signed(matrix_b[95:64]);
            low_products[95:64] = prod_lane2[31:0];

            prod_lane3 = $signed(matrix_a[127:96])  * $signed(matrix_b[127:96]);
            low_products[127:96] = prod_lane3[31:0];

            result <= {128'b0, low_products};
            done <= 1'b1;
        end else begin
            done <= 1'b0;
            result <= 256'b0;
        end
    end
endmodule

