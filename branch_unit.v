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

