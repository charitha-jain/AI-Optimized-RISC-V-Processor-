module cpu_tb;

    reg clk = 0;
    reg reset = 1;

    parameter CLK_PERIOD = 4;

    top cpu (
        .clk(clk),
        .reset(reset)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    initial begin
        $dumpfile("cpu_tb.vcd");
        $dumpvars(0, cpu);

        #(CLK_PERIOD*5);
        reset = 0;

        #(CLK_PERIOD*2000);
        $finish;
    end

endmodule

