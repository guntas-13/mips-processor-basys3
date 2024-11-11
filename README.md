# mips-processor-basys3

### Team ALUminati - ES215 Computer Organization and Architecture

## BRAM
<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/MemOrganization.png" style="width: 100%">
</div>

<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/MemReadWrite.png" style="width: 100%">
</div>

## ALU
<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/ALU.png" style="width: 100%">
</div>

<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/our_alu.png" style="width: 45%; float: left;">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/ALU_modification1.png" style="width: 35%;">
</div>

<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/ALU_waveform_after_modification_latest.png" style="width: 100%">
</div>

<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/ALU_waveform.png" style="width: 100%">
</div>

## Register File
<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/reg_run.png" style="width: 100%">
</div>

## 2x1 MUX
<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/mux_run.png" style="width: 100%">
</div>

## Factorial Waveform
<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/mips_v1/media/factorial_waveform.png" style="width: 100%">
</div>

## Code with all instructions
### Code
```
lw $s0, 0($gp)
lw $s1, 1($gp)
add $t0, $s0, $s1
beq $t0, $s0, exit
j next
next: div $s0, $s1
mflo $t1
sw $t0, 2($gp)
sw $t1, 3($gp)
jal exit
sysend
exit: jr $ra
```
### Simulation
<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/mips_v1/media/mips_v1_all_inst.png" style="width: 100%">
</div>
