OPCODES = {
    "addi": "001000", "sw": "101011", "lw": "100010", "beq": "000100",
    "j": "000010", "jal": "000011", "slti": "001010", "andi": "001100",
    "ori": "001101", "add": "000000", "sub": "000000", "and": "000000",
    "or": "000000", "nor": "000000", "slt": "000000", "sll": "000000",
    "srl": "000000", "mult": "000000", "div": "000000", "mflo": "000000",
    "mfhi": "000000", "jr": "000000", "sysend": "111111"
}

FUNCTS = {
    "add": "100000", "sub": "100010", "and": "100100", "or": "100101",
    "nor": "100111", "slt": "101010", "sll": "000000", "srl": "000010",
    "mult": "011000", "div": "011010", "mflo": "010000", "mfhi": "010010",
    "jr": "001000"
}

REGISTERS = {
    "$zero": "00000", "$at": "00001", "$v0": "00010", "$v1": "00011",
    "$a0": "00100", "$a1": "00101", "$a2": "00110", "$a3": "00111",
    "$t0": "01000", "$t1": "01001", "$t2": "01010", "$t3": "01011",
    "$t4": "01100", "$t5": "01101", "$t6": "01110", "$t7": "01111",
    "$s0": "10000", "$s1": "10001", "$s2": "10010", "$s3": "10011",
    "$s4": "10100", "$s5": "10101", "$s6": "10110", "$s7": "10111",
    "$t8": "11000", "$t9": "11001", "$k0": "11010", "$k1": "11011",
    "$gp": "11100", "$sp": "11101", "$fp": "11110", "$ra": "11111"
}

def decimal_to_binary(value, bits):
    """Convert a signed integer to binary with 'bits' length."""
    if value < 0:
        value = (1 << bits) + value
    return format(value, f'0{bits}b')

def first_pass(assembly_code):
    """First pass to collect all labels with their instruction addresses."""
    labels = {}
    instruction_count = 0

    for line in assembly_code:
        # Remove comments
        line = line.split("#")[0].strip()
        if not line:
            continue  # Skip empty lines

        # Check if the line contains a label (e.g., `label: instruction`)
        if ":" in line:
            label, instruction = line.split(":", 1)
            label = label.strip()
            instruction = instruction.strip()
            labels[label] = instruction_count  # Map label to current instruction address
            if instruction:
                instruction_count += 1  # Count the instruction after the label
        else:
            instruction_count += 1  # Regular instruction

    return labels

def second_pass(assembly_code, labels):
    """Second pass to convert assembly instructions into machine code."""
    machine_code = []
    instruction_count = 0

    for line in assembly_code:
        # Remove comments
        line = line.split("#")[0].strip()
        if not line:
            continue  # Skip empty lines

        # Ignore labels
        if ":" in line:
            _, instruction = line.split(":", 1)
            line = instruction.strip()
            if not line:
                continue  # Skip if no instruction after label

        if line:
            machine_code_line = parse_instruction(line, labels, instruction_count)
            machine_code.append(machine_code_line)
            instruction_count += 1

    return machine_code


def parse_instruction(line, labels, current_address):
    """Parse a single line of MIPS assembly code and convert it to machine code, resolving labels."""
    parts = line.strip().replace(",", "").split()
    if not parts:
        return ""  # Skip empty lines
    
    opcode = parts[0]
    bin_instruction = ""

    try:
        if opcode in OPCODES:
            opcode_bin = OPCODES[opcode]

            if opcode in {"addi", "andi", "ori", "slti"}:
                # I-Type Immediate Instruction, expecting format: `opcode rt, rs, imm`
                if len(parts) < 4:
                    raise ValueError(f"Instruction '{line}' is missing operands")
                rt = REGISTERS[parts[1]]
                rs = REGISTERS[parts[2]]
                imm = decimal_to_binary(int(parts[3]), 16)
                bin_instruction = f"{opcode_bin}{rs}{rt}{imm}"
            
            # mult and div are mult $rs, $rt and div $rs, $rt
            elif opcode in {"mult", "div"}:
                # R-Type Instruction, expecting format: `opcode rs, rt`
                if len(parts) < 3:
                    raise ValueError(f"Instruction '{line}' is missing operands")
                rs = REGISTERS[parts[1]]
                rt = REGISTERS[parts[2]]
                bin_instruction = f"{opcode_bin}{rs}{rt}0000000000{FUNCTS[opcode]}"
            
            # mflo and mfhi are mflo $rd and mfhi $rd
            elif opcode in {"mflo", "mfhi"}:
                # R-Type Instruction, expecting format: `opcode rd`
                if len(parts) < 2:
                    raise ValueError(f"Instruction '{line}' is missing operands")
                rd = REGISTERS[parts[1]]
                bin_instruction = f"{opcode_bin}0000000000{rd}00000{FUNCTS[opcode]}"

            elif opcode in {"lw", "sw"}:
                # I-Type Memory Access Instruction, expecting format: `opcode rt, offset(base)`
                if len(parts) < 3 or "(" not in parts[2]:
                    raise ValueError(f"Instruction '{line}' is missing operands or has invalid format")
                rt = REGISTERS[parts[1]]
                offset, base = parts[2].split("(")
                offset = decimal_to_binary(int(offset), 16)
                base = REGISTERS[base.strip(")")]
                bin_instruction = f"{opcode_bin}{base}{rt}{offset}"

            elif opcode == "beq":
                # I-Type Branch Instruction, expecting format: `beq rs, rt, label`
                if len(parts) < 4:
                    raise ValueError(f"Instruction '{line}' is missing operands")
                rs = REGISTERS[parts[1]]
                rt = REGISTERS[parts[2]]
                label = parts[3]
                if label in labels:
                    # Calculate offset as the relative address from the next instruction
                    offset = labels[label] - (current_address + 1)
                    imm = decimal_to_binary(offset, 16)
                else:
                    raise ValueError(f"Undefined label: {label}")
                bin_instruction = f"{opcode_bin}{rs}{rt}{imm}"
            
            elif opcode == 'sysend':
                bin_instruction = '11111100000000000000000000000000'

            elif opcode in {"j", "jal"}:
                # J-Type Jump Instruction, expecting format: `j label` or `jal label`
                if len(parts) < 2:
                    raise ValueError(f"Instruction '{line}' is missing label")
                label = parts[1]
                if label in labels:
                    address = decimal_to_binary(labels[label], 26)
                else:
                    raise ValueError(f"Undefined label: {label}")
                bin_instruction = f"{opcode_bin}{address}"

            elif opcode in FUNCTS:
                # R-Type Instruction, expecting format: `opcode rd, rs, rt`
                if opcode in {"sll", "srl"}:
                    # Shift Instructions, format: `opcode rd, rt, shamt`
                    if len(parts) < 4:
                        raise ValueError(f"Instruction '{line}' is missing operands")
                    rd = REGISTERS[parts[1]]
                    rt = REGISTERS[parts[2]]
                    shamt = decimal_to_binary(int(parts[3]), 5)
                    rs = "00000"  # Shift instructions don't use `rs`
                    funct_bin = FUNCTS[opcode]
                    bin_instruction = f"{opcode_bin}{rs}{rt}{rd}{shamt}{funct_bin}"
                elif opcode == "jr":
                    # Jump Register Instruction, format: `jr rs`
                    if len(parts) < 2:
                        raise ValueError(f"Instruction '{line}' is missing register")
                    rs = REGISTERS[parts[1]]
                    bin_instruction = f"{opcode_bin}{rs}000000000000000{FUNCTS[opcode]}"
                else:
                    # Other R-Type Instructions, expecting `opcode rd, rs, rt`
                    if len(parts) < 4:
                        raise ValueError(f"Instruction '{line}' is missing operands")
                    rd = REGISTERS[parts[1]]
                    rs = REGISTERS[parts[2]]
                    rt = REGISTERS[parts[3]]
                    shamt = "00000"  # Default shift amount
                    funct_bin = FUNCTS[opcode]
                    bin_instruction = f"{opcode_bin}{rs}{rt}{rd}{shamt}{funct_bin}"

    except KeyError as e:
        raise ValueError(f"Unknown register or instruction in line '{line}': {e}")
    except ValueError as e:
        raise ValueError(f"Invalid instruction format in line '{line}': {e}")

    return bin_instruction

# Helper function to write the machine code to an output file
def write_machine_code_to_file(machine_code, output_file):
    with open(output_file, "w") as file:
        for instruction in machine_code:
            file.write(f"{instruction}\n")


# Main function to assemble MIPS code from input file to output file
def assemble_mips(input_file, output_file):
    with open(input_file, "r") as file:
        assembly_code = file.readlines()
    
    # First pass to collect labels
    labels = first_pass(assembly_code)
    
    # Second pass to generate machine code
    machine_code = second_pass(assembly_code, labels)
    
    # Write the machine code to the output file
    write_machine_code_to_file(machine_code, output_file)