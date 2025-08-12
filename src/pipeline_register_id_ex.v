module pipeline_register_id_ex (
    input clk, reset, stall,
    input [31:0] pc_in, read_data1_in, read_data2_in, imm_in,
    input [3:0] alu_op_in,
    input [4:0] rs1_in, rs2_in, rd_in,
    input regwrite_in,
    input is_ai_in,
    input [2:0] ai_opcode_in,
    input [6:0] op_in,
    output reg [31:0] pc_out, read_data1_out, read_data2_out, imm_out,
    output reg [3:0] alu_op_out,
    output reg [4:0] rs1_out, rs2_out, rd_out,
    output reg regwrite_out,
    output reg is_ai_out,
    output reg [2:0] ai_opcode_out,
    output reg [6:0] op_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 0; read_data1_out <= 0; read_data2_out <= 0;
            imm_out <= 0; alu_op_out <= 0;
            rs1_out <= 0; rs2_out <= 0; rd_out <= 0;
            regwrite_out <= 0; is_ai_out <= 0; ai_opcode_out <= 0;
            op_out <= 0;
        end else if (!stall) begin
            pc_out <= pc_in; read_data1_out <= read_data1_in; read_data2_out <= read_data2_in;
            imm_out <= imm_in; alu_op_out <= alu_op_in;
            rs1_out <= rs1_in; rs2_out <= rs2_in; rd_out <= rd_in;
            regwrite_out <= regwrite_in; is_ai_out <= is_ai_in; ai_opcode_out <= ai_opcode_in;
            op_out <= op_in;
        end
    end

endmodule

