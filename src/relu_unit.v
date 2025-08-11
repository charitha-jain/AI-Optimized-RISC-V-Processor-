module relu_unit(
    input  [127:0] in_data,
    output [127:0] out_data
);
    assign out_data[31:0]    = (in_data[31]  == 1'b1) ? 32'b0 : in_data[31:0];
    assign out_data[63:32]   = (in_data[63]  == 1'b1) ? 32'b0 : in_data[63:32];
    assign out_data[95:64]   = (in_data[95]  == 1'b1) ? 32'b0 : in_data[95:64];
    assign out_data[127:96]  = (in_data[127] == 1'b1) ? 32'b0 : in_data[127:96];
endmodule

