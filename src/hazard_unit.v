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

