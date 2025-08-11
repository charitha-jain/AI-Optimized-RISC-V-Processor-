module top (
    input clk, reset
);

    // ============ IF Stage ============
    wire [31:0] pc_if, inst_if;
    pc pc0(.clk(clk), .reset(reset), .pc_out(pc_if));
    imem imem0(.pc(pc_if), .instruction(inst_if));

    // ============ IF/ID ============
    wire [31:0] pc_id, inst_id;
    wire stall;
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
    wire [127:0] read1_id, read2_id;
    wire [31:0] imm_id_32;
    wire [127:0] imm_id;
    assign imm_id = {96'b0, imm_id_32};
    wire [3:0] alu_op_id;
    wire regwrite_id, is_ai_id;
    wire [2:0] ai_opcode_id;
    wire [4:0] rd_wb;
    wire [127:0] writeback_data;
    wire regwrite_wb;
    register_file regs(
        .clk(clk), .reset(reset),
        .read_reg1(rs1_id), .read_reg2(rs2_id),
        .write_reg(rd_wb), .write_data(writeback_data),
        .reg_write(regwrite_wb),
        .read_data1(read1_id), .read_data2(read2_id)
    );
    immediate_generator imm(.instruction(inst_id), .immediate(imm_id_32));
    control_unit ctrl(.opcode(op_id), .reg_write(regwrite_id), .alu_op(alu_op_id));
    ai_instruction_decoder decoder(
        .instruction(inst_id),
        .is_ai_instruction(is_ai_id),
        .ai_opcode(ai_opcode_id)
    );

    // ============ ID/EX ============
    wire [31:0] pc_ex;
    wire [127:0] imm_ex, read1_ex, read2_ex;
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
        .is_ai_in(is_ai_id), .ai_opcode_in(ai_opcode_id),
        .pc_out(pc_ex), .read_data1_out(read1_ex), .read_data2_out(read2_ex),
        .imm_out(imm_ex), .alu_op_out(alu_op_ex),
        .rs1_out(rs1_ex), .rs2_out(rs2_ex), .rd_out(rd_ex),
        .regwrite_out(regwrite_ex),
        .is_ai_out(is_ai_ex), .ai_opcode_out(ai_opcode_ex)
    );

    // ============ EX Stage ============
    wire [127:0] alu_out_ex128, relu_ex, sigmoid_ex, step_ex, dot_ex;
    wire ai_start = is_ai_ex && (ai_opcode_ex == 3'b001);
    wire ai_busy, ai_done;
    reg [127:0] sel_ai_ex128;
    alu alu0(
        .input_a(read1_ex), .input_b(imm_ex), .alu_op(alu_op_ex),
        .alu_result(alu_out_ex128), .zero()
    );
    dot_product dot0(
        .vec_a(read1_ex), .vec_b(read2_ex),
        .dot_result(dot_ex)
    );
    relu_unit relu0(.in_data(read1_ex), .out_data(relu_ex));
    sigmoid_unit sig0(.in_data(read1_ex), .out_data(sigmoid_ex));
    step_function step0(.in_data(read1_ex), .out_data(step_ex));
    matrix_multiplier mm0(
        .clk(clk), .start(ai_start),
        .matrix_a({128'b0, read1_ex}), .matrix_b({128'b0, read2_ex}),
        .done(ai_done), .result()
    );
    ai_unit_controller fsm(.clk(clk), .start(ai_start), .busy(ai_busy), .done(ai_done));
    always @(*) begin
        case (ai_opcode_ex)
            3'b000: sel_ai_ex128 = {96'b0, dot_ex};
            3'b001: sel_ai_ex128 = 128'b0;
            3'b010: sel_ai_ex128 = relu_ex;
            3'b011: sel_ai_ex128 = sigmoid_ex;
            3'b100: sel_ai_ex128 = step_ex;
            default: sel_ai_ex128 = 128'b0;
        endcase
    end
    wire [127:0] final_ex128 = is_ai_ex ? sel_ai_ex128 : alu_out_ex128;

    // ============ EX/MEM ============
    wire [127:0] alu_ex_mem128;
    wire [4:0] rd_mem;
    wire regwrite_mem;
    pipeline_register_ex_mem u_exmem(
        .clk(clk), .reset(reset),
        .alu_result_in(final_ex128), .rd_in(rd_ex),
        .regwrite_in(regwrite_ex && (!ai_busy || ai_done)),
        .alu_result_out(alu_ex_mem128), .rd_out(rd_mem),
        .regwrite_out(regwrite_mem)
    );

    // ============ MEM Stage ============
    wire [127:0] mem_data128;
    data_memory dmem(
        .clk(clk),
        .address(alu_ex_mem128[31:0]),
        .write_data(read2_ex),
        .mem_write(op_ex == 7'b0100011),
        .read_data(mem_data128)
    );

    // ============ MEM/WB ============
    wire [127:0] alu_mem_wb128, mem_wb_data128;
    pipeline_register_mem_wb u_memwb(
        .clk(clk), .reset(reset),
        .dm_in(mem_data128), .alu_in(alu_ex_mem128),
        .rd_in(rd_mem), .regwrite_in(regwrite_mem),
        .dm_out(mem_wb_data128), .alu_out(alu_mem_wb128),
        .rd_out(rd_wb), .regwrite_out(regwrite_wb)
    );

    // ============ WB Stage ============
    wire is_load = (op_ex == 7'b0000011);
    assign writeback_data = is_load ? mem_wb_data128 : alu_mem_wb128;

    // ============ Hazard Logic ============
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
    assign stall = stall_ai | stall_hazard;

endmodule

