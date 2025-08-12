module top (
    input clk,
    input reset
);

    // ================= IF Stage =================
    wire [31:0] pc_if;
    wire [31:0] inst_if;
    pc pc0(.clk(clk), .reset(reset), .pc_out(pc_if));
    imem imem0(.pc(pc_if), .instruction(inst_if));

    // ================= IF/ID =================
    wire [31:0] pc_id;
    wire [31:0] inst_id;
    wire stall;
    pipeline_register_if_id u_ifid(
        .clk(clk), .reset(reset), .stall(stall),
        .pc_in(pc_if), .instruction_in(inst_if),
        .pc_out(pc_id), .instruction_out(inst_id)
    );

    // ================= ID Stage =================
    wire [4:0] rs1_id = inst_id[19:15];
    wire [4:0] rs2_id = inst_id[24:20];
    wire [4:0] rd_id  = inst_id[11:7];
    wire [6:0] op_id  = inst_id[6:0];
    wire [2:0] funct3_id = inst_id[14:12];

    wire [31:0] read1_id;
    wire [31:0] read2_id;
    wire [31:0] imm_id;
    wire [3:0] alu_op_id;
    wire regwrite_id;
    wire is_ai_id;
    wire [2:0] ai_opcode_id;

    wire [4:0] rd_wb;            // writeback destination register (from MEM/WB)
    wire [31:0] writeback_data;  // data to write back to RF
    wire regwrite_wb;            // regwrite in WB stage

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

    // ================= ID/EX =================
    wire [31:0] pc_ex;
    wire [31:0] read1_ex;
    wire [31:0] read2_ex;
    wire [31:0] imm_ex;
    wire [4:0] rs1_ex;
    wire [4:0] rs2_ex;
    wire [4:0] rd_ex;
    wire [3:0] alu_op_ex;
    wire regwrite_ex;
    wire is_ai_ex;
    wire [2:0] ai_opcode_ex;
    wire [6:0] op_ex;

    pipeline_register_id_ex u_idex(
        .clk(clk), .reset(reset), .stall(stall),
        .pc_in(pc_id), .read_data1_in(read1_id), .read_data2_in(read2_id),
        .imm_in(imm_id), .alu_op_in(alu_op_id),
        .rs1_in(rs1_id), .rs2_in(rs2_id), .rd_in(rd_id),
        .regwrite_in(regwrite_id), .is_ai_in(is_ai_id), .ai_opcode_in(ai_opcode_id),
        .op_in(op_id),
        .pc_out(pc_ex), .read_data1_out(read1_ex), .read_data2_out(read2_ex),
        .imm_out(imm_ex), .alu_op_out(alu_op_ex),
        .rs1_out(rs1_ex), .rs2_out(rs2_ex), .rd_out(rd_ex),
        .regwrite_out(regwrite_ex), .is_ai_out(is_ai_ex), .ai_opcode_out(ai_opcode_ex),
        .op_out(op_ex)
    );

    // ================= EX Stage =================
    // declare these early because forwarding mux uses alu_ex_mem / writeback_data
    wire [31:0] alu_ex_mem;
    wire [4:0]  rd_mem;
    wire        regwrite_mem;
    wire [6:0]  op_mem;

    wire [31:0] alu_out_ex;
    wire [31:0] dot_result;       // scalar dot result (user name)
    wire [31:0] relu_result;      // user-named output
    wire [31:0] sigmoid_result;   // user-named output
    wire [31:0] step_result;      // user-named output
    wire [31:0] mat_res_ex;
    wire ai_start = is_ai_ex && (ai_opcode_ex == 3'b001);
    wire ai_busy;
    wire ai_done;
    reg  [31:0] sel_ai_ex;
    wire zero_ex;

    // forwarding signals
    wire [1:0] forward_a;
    wire [1:0] forward_b;
    wire [31:0] forward_a_data;
    wire [31:0] forward_b_data;
    wire [31:0] alu_input_a;
    wire [31:0] alu_input_b;
    wire use_imm_ex = (op_ex == 7'b0010011) || (op_ex == 7'b0000011) || (op_ex == 7'b0010111) || (op_ex == 7'b0110111);

    // forwarding muxes (01 = EX/MEM, 10 = MEM/WB, 00 = ID/EX)
    assign forward_a_data = (forward_a == 2'b01) ? alu_ex_mem :
                            (forward_a == 2'b10) ? writeback_data : read1_ex;
    assign forward_b_data = (forward_b == 2'b01) ? alu_ex_mem :
                            (forward_b == 2'b10) ? writeback_data : read2_ex;

    assign alu_input_a = forward_a_data;
    assign alu_input_b = use_imm_ex ? imm_ex : forward_b_data;

    alu alu0(
        .input_a(alu_input_a),
        .input_b(alu_input_b),
        .alu_op(alu_op_ex),
        .alu_result(alu_out_ex),
        .zero(zero_ex)
    );

    dot_product dot0(.vec_a(read1_ex), .vec_b(read2_ex), .dot_result(dot_result));
    relu_unit relu0(.in_data(read1_ex), .relu_result(relu_result));
    sigmoid_unit sig0(.in_data(read1_ex), .sigmoid_result(sigmoid_result));
    step_function step0(.in_data(read1_ex), .step_result(step_result));

    matrix_multiplier mm0(
        .clk(clk),.reset(reset), .start(ai_start),
        .matrix_a(read1_ex), .matrix_b(read2_ex),
        .done(ai_done), .result(mat_res_ex)
    );
    ai_unit_controller fsm(.clk(clk), .start(ai_start), .busy(ai_busy), .done(ai_done));

    always @(*) begin
        case (ai_opcode_ex)
            3'b000: sel_ai_ex = dot_result;
            3'b001: sel_ai_ex = mat_res_ex;
            3'b010: sel_ai_ex = relu_result;
            3'b011: sel_ai_ex = sigmoid_result;
            3'b100: sel_ai_ex = step_result;
            default: sel_ai_ex = 32'b0;
        endcase
    end

    wire [31:0] final_ex = is_ai_ex ? sel_ai_ex : alu_out_ex;

    // ================= EX/MEM =================
    pipeline_register_ex_mem u_exmem(
        .clk(clk), .reset(reset),
        .alu_result_in(final_ex), .rd_in(rd_ex),
        .regwrite_in(regwrite_ex && (!ai_busy || ai_done)),
        .op_in(op_ex),
        .alu_result_out(alu_ex_mem), .rd_out(rd_mem),
        .regwrite_out(regwrite_mem), .op_out(op_mem)
    );

    // ================= MEM Stage =================
    wire [31:0] mem_data;
    wire mem_read_mem = (op_mem == 7'b0000011);

    data_memory dmem(
        .clk(clk),
        .address(alu_ex_mem),
        .write_data(forward_b_data),
        .mem_write(op_mem == 7'b0100011),
        .mem_read(mem_read_mem),
        .read_data(mem_data)
    );

    // ================= MEM/WB =================
    wire [31:0] alu_mem_wb;
    wire [31:0] mem_wb_data;
    wire [6:0] op_wb;

    pipeline_register_mem_wb u_memwb(
        .clk(clk), .reset(reset),
        .dm_in(mem_data), .alu_in(alu_ex_mem),
        .rd_in(rd_mem), .regwrite_in(regwrite_mem), .op_in(op_mem),
        .dm_out(mem_wb_data), .alu_out(alu_mem_wb),
        .rd_out(rd_wb), .regwrite_out(regwrite_wb), .op_out(op_wb)
    );

    wire is_load = (op_wb == 7'b0000011);
    assign writeback_data = is_load ? mem_wb_data : alu_mem_wb;

    // ================= Hazard & Forwarding =================
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
        .mem_read_ex(op_ex == 7'b0000011),
        .stall(stall_hazard)
    );

    forwarding_unit fwd_inst(
        .id_ex_rs1(rs1_ex), .id_ex_rs2(rs2_ex),
        .ex_mem_rd(rd_mem), .mem_wb_rd(rd_wb),
        .ex_mem_reg_write(regwrite_mem), .mem_wb_reg_write(regwrite_wb),
        .forward_a(forward_a), .forward_b(forward_b)
    );

    assign stall = stall_ai | stall_hazard;

endmodule

