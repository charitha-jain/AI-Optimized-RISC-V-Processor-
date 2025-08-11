module register_file(
    input clk,
    input reset,
    input [4:0] read_reg1, read_reg2, write_reg,
    input [127:0] write_data,
    input reg_write,
    output [127:0] read_data1, read_data2
);

    reg [127:0] registers [31:0];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 128'b0;
            end
        end else if (reg_write && (write_reg != 0)) begin
            registers[write_reg] <= write_data;
        end
    end

    assign read_data1 = registers[read_reg1];
    assign read_data2 = registers[read_reg2];

endmodule

