module pipeline_register_mem_wb (
    input clk, reset,
    input [127:0] dm_in, alu_in,
    input [4:0] rd_in,
    input regwrite_in,

    output reg [127:0] dm_out, alu_out,
    output reg [4:0] rd_out,
    output reg regwrite_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            dm_out <= 0; alu_out <= 0; rd_out <= 0; regwrite_out <= 0;
        end else begin
            dm_out <= dm_in;
            alu_out <= alu_in;
            rd_out <= rd_in;
            regwrite_out <= regwrite_in;
        end
    end

endmodule

