module imem(
    input [31:0] pc,
    output reg [31:0] instruction
);

    reg [31:0] memory [0:1023]; // Memory with 1024 words

    initial begin
        $readmemh("program.hex", memory); // Load instruction memory from .hex file
    end

    always @(pc) begin
        instruction = memory[pc >> 2]; // Word-aligned access
    end
endmodule
