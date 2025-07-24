module step_function(
    input [31:0] in_data,
    output [31:0] out_data
);
    assign out_data = (in_data[31] == 1) ? 0 : 1;
endmodule
