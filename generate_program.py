# generate_program.py
with open("program.hex", "w") as file:
    for i in range(1024):  # Generate exactly 1024 words
        file.write(f"{i:08X}\n")  # Write the value as 8-digit hex, padded
