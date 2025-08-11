module imem(
    input [31:0] pc,
    output reg [31:0] instruction
);

    reg [31:0] memory [0:1023]; // Memory with 1024 words

    `ifndef PROGRAM_HEX
        `define PROGRAM_HEX "../programs/program.hex"
    `endif

    initial begin
        $display("Loading program from: %s", `PROGRAM_HEX);
        $readmemh(`PROGRAM_HEX, memory);
    end


    always @(pc) begin
        instruction = memory[pc >> 2]; // Word-aligned access
    end
endmodule
