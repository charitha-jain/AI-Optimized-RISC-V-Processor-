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

