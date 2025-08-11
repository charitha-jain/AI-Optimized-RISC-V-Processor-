module pipeline_register_ex_mem (
    input clk, reset,
    input [127:0] alu_result_in,
    input [4:0] rd_in,
    input regwrite_in,

    output reg [127:0] alu_result_out,
    output reg [4:0] rd_out,
    output reg regwrite_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            alu_result_out <= 0; rd_out <= 0; regwrite_out <= 0;
        end else begin
            alu_result_out <= alu_result_in;
            rd_out <= rd_in;
            regwrite_out <= regwrite_in;
        end
    end

endmodule

