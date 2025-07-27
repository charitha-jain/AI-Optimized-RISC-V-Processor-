
module relu_unit(
    input [31:0] in_data,
    output [31:0] relu_result
);
    assign relu_result = (in_data[31] == 1) ? 0 : in_data;
endmodule

