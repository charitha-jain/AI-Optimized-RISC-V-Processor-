module step_function(
    input [31:0] in_data,
    output [31:0] step_result
);
    assign step_result = (in_data[31] == 1) ? 0 : 1;
endmodule
