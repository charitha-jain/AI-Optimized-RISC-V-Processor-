module matrix_multiplier(
    input clk,
    input start,
    input [255:0] matrix_a,
    input [255:0] matrix_b,
    output reg done,
    output reg [255:0] result
);
    integer i;
    reg signed [63:0] prod_lane;
    reg [127:0] low_products;

    always @(posedge clk) begin
        if (start) begin
            low_products = 128'b0;
            for (i = 0; i < 4; i = i + 1) begin
                prod_lane = $signed(matrix_a[32*i +: 32]) * $signed(matrix_b[32*i +: 32]);
                low_products[32*i +: 32] = prod_lane[31:0];
            end
	    result <= {128'b0, low_products};
            done <= 1'b1;
        end else begin
            done <= 1'b0;
            result <= 256'b0;
        end
    end

endmodule

