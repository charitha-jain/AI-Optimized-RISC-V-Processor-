module ai_result_mux(
    input [127:0] alu_result,
    input [127:0] ai_result,
    input use_ai_result,
    output [127:0] final_result
);
    assign final_result = use_ai_result ? ai_result : alu_result;
endmodule

