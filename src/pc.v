module pc(
    input clk,
    input reset,
    output reg [31:0] pc_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 32'b0;
        end else begin
            pc_out <= pc_out + 4; // Increment PC by 4 each clock cycle
        end
    end
endmodule
