# AI-Optimized RISC-V Processor 🚀

This project implements a pipelined RISC-V CPU with custom AI-specific instruction support including:
- Matrix multiplication
- Dot product
- Sigmoid/ReLU/Step functions

## Tools Used
- Icarus Verilog
- GTKWave
- Synopsys & Cadence (college-provided tools)

## How to Run
```bash
iverilog -o cpu_tb.vvp cpu_tb.v <other .v files>
vvp cpu_tb.vvp
gtkwave cpu_tb.vcd
