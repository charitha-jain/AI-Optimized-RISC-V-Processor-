
module ai_result_mux(
    input [31:0] alu_result,
    input [31:0] ai_result,
    input use_ai_result,
    output [31:0] final_result
);
    assign final_result = use_ai_result ? ai_result : alu_result;
endmodule
