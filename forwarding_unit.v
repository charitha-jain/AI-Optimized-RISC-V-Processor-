module forwarding_unit(
    input [4:0] id_ex_rs1, id_ex_rs2,
    input [4:0] ex_mem_rd, mem_wb_rd,
    input ex_mem_reg_write, mem_wb_reg_write,
    output reg forward_a, forward_b
);
    always @(*) begin
        forward_a = 0;
        forward_b = 0;
        if (ex_mem_reg_write && ex_mem_rd != 0 && ex_mem_rd == id_ex_rs1)
            forward_a = 1;
        if (mem_wb_reg_write && mem_wb_rd != 0 && mem_wb_rd == id_ex_rs2)
            forward_b = 1;
    end
endmodule
