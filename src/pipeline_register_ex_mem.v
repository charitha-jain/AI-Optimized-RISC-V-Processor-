module pipeline_register_ex_mem (
    input clk, reset,
    input [31:0] alu_result_in,
    input [4:0] rd_in,
    input regwrite_in,
    input [6:0] op_in,
    output reg [31:0] alu_result_out,
    output reg [4:0] rd_out,
    output reg regwrite_out,
    output reg [6:0] op_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            alu_result_out <= 0; rd_out <= 0; regwrite_out <= 0; op_out <= 0;
        end else begin
            alu_result_out <= alu_result_in;
            rd_out <= rd_in;
            regwrite_out <= regwrite_in;
            op_out <= op_in;
        end
    end

endmodule

