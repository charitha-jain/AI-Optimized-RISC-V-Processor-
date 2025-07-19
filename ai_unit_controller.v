module ai_unit_controller(
    input clk,
    input start,
    output reg busy,
    output reg done
);
    reg [1:0] state;
    always @(posedge clk) begin
        case (state)
            0: if (start) begin busy <= 1; state <= 1; end
            1: begin done <= 1; busy <= 0; state <= 0; end
        endcase
    end
endmodule
