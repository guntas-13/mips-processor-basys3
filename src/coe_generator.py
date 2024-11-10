mem_words = 51200  # Total memory words

with open("mips_machine_code.txt", "r") as mc_file:
    machine_code_lines = [line.strip() for line in mc_file.readlines()]

text_segment_size = len(machine_code_lines)  # Number of machine code instructions
data_segment_start = 6300  # Start of data segment

with open("data.coe", "w") as f:
    f.write("memory_initialization_radix=2;\n")
    f.write("memory_initialization_vector=\n")
    
    for i in range(mem_words):
        if i < text_segment_size:
            # Write the machine code instructions in the text segment
            f.write(f"{machine_code_lines[i]},\n")
        elif i == text_segment_size:
            # the terminating instruction with opcode = 111111
            f.write("11111100000000000000000000000000,\n")
        elif i < data_segment_start:
            # Fill with zeros between text and data segments if any space remains
            f.write("00000000000000000000000000000000,\n")
        elif i == data_segment_start:
            # Begin the data segment at the specified start index (e.g., 6300)
            f.write("00000000000000000000000000000001,\n")
        elif i < mem_words - 1:
            # Continue filling with zeros in the remaining memory
            f.write("00000000000000000000000000000000,\n")
        else:
            # Last memory word terminates with a semicolon
            f.write("00000000000000000000000000000000;")