module cpu_tb;

    reg clk = 0;
    reg reset = 1;

    // Instantiate the DUT (Device Under Test)
    top cpu (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation
    always #2 clk = ~clk;

    initial begin
        $dumpfile("cpu_tb.vcd");
        $dumpvars(0, cpu_tb);

        // Reset pulse sequence
        #30;
        reset = 0;
        #500;
        reset = 1;
        #500;
        reset = 0;
        #500;
        $finish;
    end

endmodule
