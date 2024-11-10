# Define the opcodes and registers dictionary
OPCODES = {
    "addi": "001000", "sw": "101011", "slti": "001010", "beq": "000100",
    "jr": "001000", "jal": "000011", "lw": "100011", "mul": "000000", "j": "000010",
    "sll": "000000", "srl": "000000", "mfhi": "000000", "mflo": "000000",
    "mult": "000000", "div": "000000", "add": "000000", "sub": "000000",
    "and": "000000", "or": "000000", "nor": "000000", "slt": "000000",
    "andi": "001100", "ori": "001101"
}

REGISTERS = {
    "$zero": "00000", "$v0": "00010", "$a0": "00100", "$t0": "01000", "$sp": "11101", "$ra": "11111",
    "$v1": "00011", "$a1": "00101", "$a2": "00110", "$a3": "00111", "$t1": "01001", "$t2": "01010",
    "$t3": "01011", "$t4": "01100", "$t5": "01101", "$t6": "01110", "$t7": "01111", "$s0": "10000",
    "$s1": "10001", "$s2": "10010", "$s3": "10011", "$s4": "10100", "$s5": "10101", "$s6": "10110",
    "$s7": "10111", "$t8": "11000", "$t9": "11001", "$k0": "11010", "$k1": "11011", "$gp": "11100",
    "$sp": "11101", "$fp": "11110", "$ra": "11111"
}

def parse_register(reg):
    """Converts register name to binary code."""
    reg = reg.strip(",")  # Remove any trailing commas
    return REGISTERS[reg]

def to_binary(value, bits):
    """Converts an integer to a two's complement binary string of a given bit length."""
    if value < 0:
        value = (1 << bits) + value
    return f"{value:0{bits}b}"

def assemble_r_type(opcode, rs, rt, rd, shamt, funct):
    """Assembles an R-type instruction."""
    return opcode + rs + rt + rd + shamt + funct

def assemble_i_type(opcode, rs, rt, immediate):
    """Assembles an I-type instruction."""
    return opcode + rs + rt + to_binary(immediate, 16)

def assemble_j_type(opcode, address):
    """Assembles a J-type instruction."""
    return opcode + f"{address:026b}"

def assemble_mips_instruction(instruction, labels=None, program_counter=None):
    """Converts a single MIPS instruction to machine code, resolving labels if needed."""
    parts = instruction.split()
    instr = parts[0]

    if instr == "addi":
        opcode = OPCODES[instr]
        rt = parse_register(parts[1])
        rs = parse_register(parts[2])
        immediate = int(parts[3])
        return assemble_i_type(opcode, rs, rt, immediate)

    elif instr == "sw" or instr == "lw":
        opcode = OPCODES[instr]
        rt = parse_register(parts[1])
        offset, rs = parts[2].replace(")", "").split("(")
        offset = int(offset)
        rs = parse_register(rs)
        return assemble_i_type(opcode, rs, rt, offset)

    elif instr == "slti" or instr == "beq":
        opcode = OPCODES[instr]
        rs = parse_register(parts[1])
        rt = parse_register(parts[2])
        immediate = int(parts[3]) if parts[3].isdigit() else (labels[parts[3]] - (program_counter + 4)) // 4
        return assemble_i_type(opcode, rs, rt, immediate)

    elif instr == "jal" or instr == "j":
        opcode = OPCODES[instr]
        label = parts[1]
        address = labels[label] // 4
        return assemble_j_type(opcode, address)

    elif instr == "jr":
        opcode = OPCODES[instr]
        rs = parse_register(parts[1])
        return assemble_r_type("000000", rs, "00000", "00000", "00000", "001000")

    elif instr == "mul":
        rd = parse_register(parts[1])
        rs = parse_register(parts[2])
        rt = parse_register(parts[3])
        return assemble_r_type("000000", rs, rt, rd, "00000", "011000")

    else:
        raise ValueError(f"Instruction {instr} is not supported.")

def assemble_mips_with_labels(instructions):
    """Converts MIPS code with labels to machine code."""
    labels = {}
    program_counter = 0x110000  # Starting address for instructions

    # First pass: Collect labels
    for line in instructions:
        if ':' in line:
            label, instruction = line.split(':', 1)
            labels[label.strip()] = program_counter
            line = instruction.strip()
            if line:  # Increment for an instruction after the label
                program_counter += 4
        else:
            program_counter += 4

    # Second pass: Generate machine code
    machine_code_output = []
    program_counter = 0x110000

    for line in instructions:
        if ':' in line:
            label, line = line.split(':', 1)
            line = line.strip()
            if not line:
                continue  # Skip if no instruction after label

        if line:  # Only process non-empty instructions
            machine_code = assemble_mips_instruction(line, labels, program_counter)
            machine_code_output.append(f"{line} => {machine_code}")
            program_counter += 4

    return machine_code_output

def read_mips_from_file(filename):
    """Reads MIPS instructions from a .txt file."""
    with open(filename, 'r') as file:
        instructions = [line.strip() for line in file if line.strip()]
    return instructions

def main():
    choice = input("Enter 1 to input MIPS code manually, 2 to load from a file: ")

    if choice == "1":
        print("Enter your MIPS code (type 'END' to finish):")
        instructions = []
        while True:
            line = input()
            if line.strip().upper() == "END":
                break
            instructions.append(line)

    elif choice == "2":
        filename = input("Enter the filename (with .txt extension) containing MIPS instructions: ")
        instructions = read_mips_from_file(filename)

    else:
        print("Invalid choice.")
        return

    machine_code_output = assemble_mips_with_labels(instructions)

    for line in machine_code_output:
        print(line)

if __name__ == "__main__":
    main()