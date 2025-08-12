+incdir+../src
+incdir+../testbenches
+define+PROGRAM_HEX=\"../programs/program.hex\"

# RTL files
../src/pc.v
../src/imem.v
../src/data_memory.v
../src/register_file.v
../src/immediate_generator.v
../src/control_unit.v
../src/alu.v
../src/alu_control.v
../src/branch_unit.v
../src/hazard_unit.v
../src/forwarding_unit.v
../src/pipeline_register_if_id.v
../src/pipeline_register_id_ex.v
../src/pipeline_register_ex_mem.v
../src/pipeline_register_mem_wb.v
../src/ai_instruction_decoder.v
../src/matrix_multiplier.v
../src/dot_product.v
../src/step_function.v
../src/relu_unit.v
../src/sigmoid_unit.v
../src/ai_unit_controller.v
../src/ai_result_mux.v
../src/ai_test_vectors.v
../src/top.v

# Testbench files
../testbenches/cpu_tb.v

# Path for program.hex
+define+PROGRAM_PATH=\"../programs/program.hex\"
