module register_file(
    input clk,
    input reset,
    input [4:0] read_reg1, read_reg2, write_reg,
    input [31:0] write_data,
    input reg_write,
    output [31:0] read_data1, read_data2
);

    reg [31:0] registers [31:0];
    integer i; // Declare 'i' outside of the always block for Verilog compliance

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all registers to 0
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (reg_write && write_reg != 0) begin
            registers[write_reg] <= write_data;
        end
    end

    assign read_data1 = registers[read_reg1];
    assign read_data2 = registers[read_reg2];

endmodule
