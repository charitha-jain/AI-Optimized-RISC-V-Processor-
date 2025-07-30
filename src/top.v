module top (
    input clk, reset
);

    // ============ IF Stage ============
    wire [31:0] pc_if, inst_if;
    pc pc0(.clk(clk), .reset(reset), .pc_out(pc_if));
    imem imem0(.pc(pc_if), .instruction(inst_if));

    // ============ IF/ID Pipeline ============
    wire [31:0] pc_id, inst_id;
    pipeline_register_if_id u_ifid(
        .clk(clk), .reset(reset), .stall(stall),
        .pc_in(pc_if), .instruction_in(inst_if),
        .pc_out(pc_id), .instruction_out(inst_id)
    );

    // ============ ID Stage ============
    wire [4:0] rs1_id = inst_id[19:15];
    wire [4:0] rs2_id = inst_id[24:20];
    wire [4:0] rd_id  = inst_id[11:7];
    wire [6:0] op_id  = inst_id[6:0];
    wire [2:0] funct3_id = inst_id[14:12];

    wire [31:0] read1_id, read2_id, imm_id;
    wire [3:0] alu_op_id;
    wire regwrite_id, is_ai_id;
    wire [2:0] ai_opcode_id;

    register_file regs(
        .clk(clk), .reset(reset),
        .read_reg1(rs1_id), .read_reg2(rs2_id),
        .write_reg(rd_wb), .write_data(writeback_data),
        .reg_write(regwrite_wb),
        .read_data1(read1_id), .read_data2(read2_id)
    );

    immediate_generator imm(.instruction(inst_id), .immediate(imm_id));
    control_unit ctrl(.opcode(op_id), .reg_write(regwrite_id), .alu_op(alu_op_id));
    ai_instruction_decoder decoder(
        .instruction(inst_id),
        .is_ai_instruction(is_ai_id),
        .ai_opcode(ai_opcode_id)
    );

    // ============ ID/EX Pipeline ============
    wire [31:0] pc_ex, read1_ex, read2_ex, imm_ex;
    wire [4:0] rs1_ex, rs2_ex, rd_ex;
    wire [3:0] alu_op_ex;
    wire regwrite_ex, is_ai_ex;
    wire [2:0] ai_opcode_ex;
    wire [6:0] op_ex;

    pipeline_register_id_ex u_idex(
        .clk(clk), .reset(reset),
        .pc_in(pc_id), .read_data1_in(read1_id), .read_data2_in(read2_id),
        .imm_in(imm_id), .alu_op_in(alu_op_id),
        .rs1_in(rs1_id), .rs2_in(rs2_id), .rd_in(rd_id),
        .regwrite_in(regwrite_id),
        .is_ai_in(is_ai_id), .ai_opcode_in(ai_opcode_id), .op_in(op_id),

        .op_out(op_ex), .pc_out(pc_ex), .read_data1_out(read1_ex), .read_data2_out(read2_ex),
        .imm_out(imm_ex), .alu_op_out(alu_op_ex),
        .rs1_out(rs1_ex), .rs2_out(rs2_ex), .rd_out(rd_ex),
        .regwrite_out(regwrite_ex),
        .is_ai_out(is_ai_ex), .ai_opcode_out(ai_opcode_ex)
    );

    // ============ EX Stage ============
    wire [31:0] alu_out_ex, dot_ex, relu_ex, sigmoid_ex, step_ex;
    wire [255:0] mat_res_ex;
    wire ai_start = is_ai_ex && (ai_opcode_ex == 3'b001);
    wire ai_busy, ai_done;
    reg [31:0] sel_ai_ex;

    alu alu0(.input_a(read1_ex), .input_b(imm_ex), .alu_op(alu_op_ex), .alu_result(alu_out_ex));
    dot_product dot0(.vec_a(read1_ex[127:0]), .vec_b(read2_ex[127:0]), .dot_result(dot_ex));
    relu_unit relu0(.in_data(read1_ex), .relu_result(relu_ex));
    sigmoid_unit sig0(.in_data(read1_ex), .sigmoid_result(sigmoid_ex));
    step_function step0(.in_data(read1_ex), .step_result(step_ex));

    matrix_multiplier mm0(
        .clk(clk), .start(ai_start),
        .matrix_a({224'b0, read1_ex}), .matrix_b({224'b0, read2_ex}),
        .done(ai_done), .result(mat_res_ex)
    );
    ai_unit_controller fsm(.clk(clk), .start(ai_start), .busy(ai_busy), .done(ai_done));

    always @(*) begin
        case (ai_opcode_ex)
            3'b000: sel_ai_ex = dot_ex;
            3'b001: sel_ai_ex = mat_res_ex[31:0];
            3'b010: sel_ai_ex = relu_ex;
            3'b011: sel_ai_ex = sigmoid_ex;
            3'b100: sel_ai_ex = step_ex;
            default: sel_ai_ex = 32'b0;
        endcase
    end

    wire [31:0] final_ex = is_ai_ex ? sel_ai_ex : alu_out_ex;

    // ============ EX/MEM Pipeline ============
    wire [31:0] alu_ex_mem;
    wire [4:0] rd_mem;
    wire regwrite_mem;

    pipeline_register_ex_mem u_exmem(
        .clk(clk), .reset(reset),
        .alu_result_in(final_ex), .rd_in(rd_ex),
        .regwrite_in(regwrite_ex && (!ai_busy || ai_done)),

        .alu_result_out(alu_ex_mem), .rd_out(rd_mem),
        .regwrite_out(regwrite_mem)
    );

    // ============ MEM Stage ============
    wire [31:0] mem_data;
    data_memory dmem(
        .clk(clk),
        .address(alu_ex_mem),
        .write_data(read2_ex),
        .mem_write(op_ex == 7'b0100011),
        .read_data(mem_data)
    );

    // ============ MEM/WB Pipeline ============
    wire [31:0] alu_mem_wb, mem_wb_data;
    wire [4:0] rd_wb;
    wire regwrite_wb;

    pipeline_register_mem_wb u_memwb(
        .clk(clk), .reset(reset),
        .dm_in(mem_data), .alu_in(alu_ex_mem),
        .rd_in(rd_mem), .regwrite_in(regwrite_mem),

        .dm_out(mem_wb_data), .alu_out(alu_mem_wb),
        .rd_out(rd_wb), .regwrite_out(regwrite_wb)
    );

    // ============ WB Stage ============
    wire is_load = (op_ex == 7'b0000011);
    wire [31:0] writeback_data = is_load ? mem_wb_data : alu_mem_wb;

    // ============ Hazard Logic (optional) ============
    wire stall_ai = ai_busy && !ai_done;
    wire stall_hazard;

    hazard_unit hz_inst(
                        .rs1_id(rs1_id),
                        .rs2_id(rs2_id),
                        .rd_ex(rd_ex),
                        .rd_mem(rd_mem),
                        .rd_wb(rd_wb),
                        .regwrite_ex(regwrite_ex),
                        .regwrite_mem(regwrite_mem),
                        .regwrite_wb(regwrite_wb),
                        .stall(stall_hazard)
    );

    wire stall = stall_ai | stall_hazard;

endmodule
