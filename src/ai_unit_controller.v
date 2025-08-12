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

