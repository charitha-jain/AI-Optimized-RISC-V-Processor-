module matrix_multiplier (
    input clk,
    input reset,
    input start,
    input [31:0] matrix_a,
    input [31:0] matrix_b,
    output reg done,
    output reg [31:0] result
);

    reg busy;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            result <= 32'b0;
            done   <= 1'b0;
            busy   <= 1'b0;
        end else begin
            if (start && !busy) begin
                busy   <= 1'b1;
                done   <= 1'b0;
                result <= $signed(matrix_a) * $signed(matrix_b);
            end else if (busy) begin
                done <= 1'b1;
                busy <= 1'b0;
            end else begin
                done <= 1'b0;
            end
        end
    end
endmodule

