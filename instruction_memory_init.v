module instruction_memory_init(
    input [7:0] addr,       // Address input to specify which memory location to read
    output [31:0] memory_data  // Output memory data
);
    reg [31:0] mem_array [0:1023];  // Internal memory array

    // Initial block to read memory from a hex file
    initial begin
        $readmemh("program.hex", mem_array);  // Initialize memory from file
    end

    // Assign the memory data based on the address
    assign memory_data = mem_array[addr];  // Dynamically access memory based on addr
endmodule
