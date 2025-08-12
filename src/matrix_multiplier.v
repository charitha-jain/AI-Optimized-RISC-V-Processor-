module matrix_multiplier(
    input clk,
    input start,
    input [31:0] a,
    input [31:0] b,
    output reg done,
    output reg [31:0] result
);
    always @(posedge clk) begin
        if (start) begin
            result <= a * b;
            done <= 1;
        end else begin
            done <= 0;
        end
    end
endmodule

