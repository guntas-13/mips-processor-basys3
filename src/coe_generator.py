mem_words = 51200
with open("data.coe", "w") as f:
    f.write("memory_initialization_radix=2;\n")
    f.write("memory_initialization_vector=\n")
    for i in range(mem_words):
        if i == mem_words - 1:
            f.write("00000000000000000000000000000000;")
        elif i == 0:
            f.write("10001111100010000000000000000000,\n") # lw $t0, 0($gp)
        elif i == 6300:
            f.write("00000000000000000000000000000001,\n")
        else :
            f.write("00000000000000000000000000000000,\n")
f.close()