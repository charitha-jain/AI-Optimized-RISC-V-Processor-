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

