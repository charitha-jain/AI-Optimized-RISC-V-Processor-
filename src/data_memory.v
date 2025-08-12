module data_memory(
    input clk,
    input [31:0] address,
    input [31:0] write_data,
    input mem_write,
    input mem_read,
    output reg [31:0] read_data
);
    reg [7:0] mem [0:1023];
    integer i;
    initial for (i=0; i<1024; i=i+1) mem[i] = 0;

    always @(posedge clk) begin
        if (mem_write) begin
            mem[address] <= write_data[7:0];
            mem[address+1] <= write_data[15:8];
            mem[address+2] <= write_data[23:16];
            mem[address+3] <= write_data[31:24];
        end
        if (mem_read) begin
            read_data <= {mem[address+3], mem[address+2], mem[address+1], mem[address]};
        end else begin
            read_data <= 32'd0;
        end
    end
endmodule

