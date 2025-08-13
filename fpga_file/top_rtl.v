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


module imem(
    input [31:0] pc,
    output reg [31:0] instruction
);

    reg [31:0] memory [0:1023]; // Memory with 1024 words

    initial begin
        $readmemh("program.hex", memory); // Load instruction memory from .hex file
    end

    always @(pc) begin
        instruction = memory[pc >> 2]; // Word-aligned access
    end
endmodule


module data_memory(
    input clk,
    input [31:0] address,
    input [31:0] write_data,
    input mem_write,
    input mem_read,
    output reg [31:0] read_data
);
    reg [7:0] mem [0:1023];
    integer i;
    initial for (i=0; i<1024; i=i+1) mem[i] = 0;

    always @(posedge clk) begin
        if (mem_write) begin
            mem[address] <= write_data[7:0];
            mem[address+1] <= write_data[15:8];
            mem[address+2] <= write_data[23:16];
            mem[address+3] <= write_data[31:24];
        end
        if (mem_read) begin
            read_data <= {mem[address+3], mem[address+2], mem[address+1], mem[address]};
        end else begin
            read_data <= 32'd0;
        end
    end
endmodule



module register_file(
    input clk,
    input reset,
    input [4:0] read_reg1, read_reg2, write_reg,
    input [31:0] write_data,
    input reg_write,
    output [31:0] read_data1, read_data2
);
    reg [31:0] registers [31:0];
    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) registers[i] <= 32'b0;
        end else if (reg_write && (write_reg != 0)) begin
            registers[write_reg] <= write_data;
        end
        registers[0] <= 32'b0;
    end

    assign read_data1 = registers[read_reg1];
    assign read_data2 = registers[read_reg2];
endmodule



module immediate_generator(
    input [31:0] instruction,
    output reg [31:0] immediate
);
    always @(*) begin
        case (instruction[6:0])
            7'b0010011, 7'b0000011: // I-type
                immediate = {{20{instruction[31]}}, instruction[31:20]};
            7'b0100011: // S-type
                immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            7'b1100011: // B-type
                immediate = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            7'b0110111, 7'b0010111: // U-type
                immediate = {instruction[31:12], 12'b0};
            7'b1101111: // J-type
                immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            default:
                immediate = 32'b0;
        endcase
    end
endmodule



module control_unit(
    input [6:0] opcode,
    output reg reg_write,
    output reg [3:0] alu_op
);

    always @(*) begin
        case (opcode)
            7'b0110011: begin // R-type instructions
                reg_write = 1;
                alu_op = 4'b0000; // ADD
            end
            7'b0000011: begin // I-type load instructions
                reg_write = 1;
                alu_op = 4'b0001; // LOAD
            end
            7'b1100011: begin // Branch instructions
                reg_write = 0;
                alu_op = 4'b0010; // Branch
            end
            default: begin
                reg_write = 0;
                alu_op = 4'b0000; // Default to ADD
            end
        endcase
    end
endmodule



module alu(
    input [31:0] input_a,
    input [31:0] input_b,
    input [3:0] alu_op,
    output reg [31:0] alu_result,
    output zero
);
    always @(*) begin
        case (alu_op)
            4'b0000: alu_result = input_a + input_b;        // ADD
            4'b0001: alu_result = input_a - input_b;        // SUB
            4'b0010: alu_result = input_a & input_b;        // AND
            4'b0011: alu_result = input_a | input_b;        // OR
            4'b0100: alu_result = input_a ^ input_b;        // XOR
            4'b0101: alu_result = input_a << input_b[4:0];  // SLL
            4'b0110: alu_result = input_a >> input_b[4:0];  // SRL
            4'b0111: alu_result = ($signed(input_a) < $signed(input_b)) ? 32'd1 : 32'd0; // SLT
            4'b1000: alu_result = (input_a < input_b) ? 32'd1 : 32'd0; // SLTU
            default: alu_result = 32'd0;
        endcase
    end
    assign zero = (alu_result == 0);
endmodule




module alu_control(
    input [2:0] funct3,
    input [6:0] funct7,
    input [1:0] alu_op,
    output reg [3:0] alu_ctrl
);
    always @(*) begin
        case (alu_op)
            2'b10: begin
                case (funct3)
                    3'b000: alu_ctrl = (funct7 == 7'b0100000) ? 4'b0001 : 4'b0000;
                    3'b111: alu_ctrl = 4'b0010;
                    3'b110: alu_ctrl = 4'b0011;
                    3'b100: alu_ctrl = 4'b0100;
                    default: alu_ctrl = 4'b0000;
                endcase
            end
            default: alu_ctrl = 4'b0000;
        endcase
    end
endmodule



module branch_unit(
    input [31:0] input_a,
    input [31:0] input_b,
    input [2:0] branch_type,
    output reg branch_taken
);
    always @(*) begin
        case (branch_type)
            3'b000: branch_taken = (input_a == input_b); // BEQ
            3'b001: branch_taken = (input_a != input_b); // BNE
            3'b100: branch_taken = (input_a < input_b);  // BLT
            3'b101: branch_taken = (input_a >= input_b); // BGE
            default: branch_taken = 0;
        endcase
    end
endmodule



module hazard_unit(
    input [4:0] rs1_id, rs2_id,
    input [4:0] rd_ex, rd_mem, rd_wb,
    input regwrite_ex, regwrite_mem, regwrite_wb,
    input mem_read_ex,
    output reg stall
);
    always @(*) begin
        stall = 0;
        // EX-stage hazard (including load-use)
        if (regwrite_ex && (rd_ex != 0) &&
            ((rd_ex == rs1_id) || (rd_ex == rs2_id))) begin
            stall = 1;
        end
        // load-use: if EX is a load and its dest matches ID source
        if (mem_read_ex && (rd_ex != 0) &&
            ((rd_ex == rs1_id) || (rd_ex == rs2_id))) begin
            stall = 1;
        end
        // MEM/WB hazards (conservative)
        if (regwrite_mem && (rd_mem != 0) &&
            ((rd_mem == rs1_id) || (rd_mem == rs2_id))) begin
            stall = 1;
        end
        if (regwrite_wb && (rd_wb != 0) &&
            ((rd_wb == rs1_id) || (rd_wb == rs2_id))) begin
            stall = 1;
        end
    end
endmodule



module forwarding_unit(
    input [4:0] id_ex_rs1, id_ex_rs2,
    input [4:0] ex_mem_rd, mem_wb_rd,
    input ex_mem_reg_write, mem_wb_reg_write,
    output reg [1:0] forward_a, forward_b
);
    // 00 - use register file / ID-EX value
    // 01 - forward from EX/MEM
    // 10 - forward from MEM/WB
    always @(*) begin
        forward_a = 2'b00;
        forward_b = 2'b00;
        if (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs1))
            forward_a = 2'b01;
        else if (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == id_ex_rs1))
            forward_a = 2'b10;

        if (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs2))
            forward_b = 2'b01;
        else if (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == id_ex_rs2))
            forward_b = 2'b10;
    end
endmodule



module pipeline_register_if_id(
    input clk,
    input reset,
    input stall,
    input [31:0] pc_in,
    input [31:0] instruction_in,
    output reg [31:0] pc_out,
    output reg [31:0] instruction_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 32'b0;
            instruction_out <= 32'b0;
        end else if (!stall) begin
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
    end

endmodule



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



module pipeline_register_mem_wb (
    input clk, reset,
    input [31:0] dm_in, alu_in,
    input [4:0] rd_in,
    input regwrite_in,
    input [6:0] op_in,
    output reg [31:0] dm_out, alu_out,
    output reg [4:0] rd_out,
    output reg regwrite_out,
    output reg [6:0] op_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            dm_out <= 0; alu_out <= 0; rd_out <= 0; regwrite_out <= 0; op_out <= 0;
        end else begin
            dm_out <= dm_in; alu_out <= alu_in; rd_out <= rd_in;
            regwrite_out <= regwrite_in;
            op_out <= op_in;
        end
    end

endmodule



module ai_instruction_decoder(
    input [31:0] instruction,
    output reg is_ai_instruction,
    output reg [2:0] ai_opcode
);
    always @(*) begin
        if (instruction[6:0] == 7'b0001011) begin
            is_ai_instruction = 1;
            ai_opcode = instruction[14:12];
        end else begin
            is_ai_instruction = 0;
            ai_opcode = 0;
        end
    end
endmodule



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



module dot_product (
    input  wire [31:0] vec_a,
    input  wire [31:0] vec_b,
    output reg  [31:0] dot_result
);
    reg [63:0] product;
    always @(*) begin
        product = $signed(vec_a) * $signed(vec_b); // signed multiply; change if unsigned
        dot_result = product[31:0]; // truncate to 32 bits
    end
endmodule



module step_function(
    input [31:0] in_data,
    output [31:0] step_result
);
    assign step_result = (in_data[31] == 1) ? 0 : 1;
endmodule



module relu_unit(
    input [31:0] in_data,
    output [31:0] relu_result
);
    assign relu_result = (in_data[31] == 1) ? 0 : in_data;
endmodule



module sigmoid_unit(
    input [31:0] in_data,
    output [31:0] sigmoid_result
);
    assign sigmoid_result = in_data[31] ? 0 : 32'h3F800000; // Placeholder: 0 or 1
endmodule



module ai_unit_controller(
    input clk,
    input start,
    output reg busy,
    output reg done
);
    reg state;
    always @(posedge clk) begin
        if (!busy && start) begin
            busy <= 1;
            done <= 0;
            state <= 1;
        end else if (busy) begin
            done <= 1;
            busy <= 0;
            state <= 0;
        end else begin
            done <= 0;
            busy <= 0;
        end
    end
endmodule




module ai_result_mux(
    input [31:0] alu_result,
    input [31:0] ai_result,
    input use_ai_result,
    output [31:0] final_result
);
    assign final_result = use_ai_result ? ai_result : alu_result;
endmodule


module ai_test_vectors(
    output reg [255:0] matrix_a,
    output reg [255:0] matrix_b
);
    initial begin
        matrix_a = 256'h0101010101010101010101010101010101010101010101010101010101010101;
        matrix_b = 256'h0202020202020202020202020202020202020202020202020202020202020202;
    end
endmodule


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



