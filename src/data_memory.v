module data_memory(
    input  clk,
    input  [31:0] address,
    input  [127:0] write_data,
    input  mem_write,         
    output reg [127:0] read_data
);

    reg [127:0] mem [0:1023];
    integer i;
    wire [31:0] word_index = address[31:4];

    initial begin
        for (i = 0; i < 1024; i = i + 1) mem[i] = 128'b0;
    end

    always @(posedge clk) begin
        if (mem_write) begin
            // write full 128-bit word at aligned address
            mem[word_index] <= write_data;
        end
        read_data <= mem[word_index];
    end

endmodule

