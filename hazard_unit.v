module hazard_unit(
    input [4:0] rs1_id, rs2_id,           // From ID stage
    input [4:0] rd_ex, rd_mem, rd_wb,     // From EX, MEM, WB stages
    input regwrite_ex, regwrite_mem, regwrite_wb,
    output reg stall
);
    always @(*) begin
        stall = 0;
        if ((regwrite_ex && (rd_ex != 0) &&
             ((rd_ex == rs1_id) || (rd_ex == rs2_id))) ||

            (regwrite_mem && (rd_mem != 0) &&
             ((rd_mem == rs1_id) || (rd_mem == rs2_id))) ||

            (regwrite_wb && (rd_wb != 0) &&
             ((rd_wb == rs1_id) || (rd_wb == rs2_id)))) begin
            stall = 1;
        end
    end
endmodule
