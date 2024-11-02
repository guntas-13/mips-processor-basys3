mem_words = 51200
with open("data.coe", "w") as f:
    f.write("memory_initialization_radix=10;\n")
    f.write("memory_initialization_vector=\n")
    for i in range(mem_words):
        if i == mem_words - 1:
            f.write(f"{i};")
        else:
            f.write(f"{i},\n")
f.close()