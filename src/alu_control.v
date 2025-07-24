
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

