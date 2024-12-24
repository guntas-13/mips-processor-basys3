# mips-processor-basys3

# Team ALUminati - ES215 Computer Organization and Architecture

The processor is implemented on the **Basys3 FPGA board**. The datapath has been modified carefully to expand the ISA - allowing _recursive programs_ to run as well.

Check out the final report **[here](https://github.com/guntas-13/mips-processor-basys3/blob/master/Final%20Report%20Team22%20-%20ALUminati.pdf)** and the final presentation **[here](https://github.com/guntas-13/mips-processor-basys3/blob/master/Final%20Presentation%20Team22%20-%20ALUminati.pdf)**, it includes video demonstrations as well.

Some MIPS assembly codes are available in the **[codes](https://github.com/guntas-13/mips-processor-basys3/tree/master/codes)** directory. The **[utils](https://github.com/guntas-13/mips-processor-basys3/tree/master/utils)** directory contains a Python script to convert human-readable MIPS assembly code to machine code.

In the **`utils`** directory, write your human-readable MIPS assembly code in **`mips_instructions.txt`** run **`coe_generator.py`**:

```console
python3 ./utils/coe_generator.py
```

**`data.coe`** will be ready to be used in the FPGA and even the 32-bit machine code will be available in **`mips_machine_code.txt`**.

## Versions of the Processor

**`mips_v3.2`** is the stable and FPGA-implementation ready version of the MIPS processor.
<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/Illustration.png" style="width: 80%">
</div>

## Modified Datapath

<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/DataPath.png" style="width: 80%">
</div>

## The FSM

<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/FSM.png" style="width: 50%">
</div>

## The Complete ISA

<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/master/media/ISA.png" style="width: 70%">
</div>

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

## Factorial Waveform

<div align = "center">
    <img src = "https://github.com/guntas-13/mips-processor-basys3/blob/mips_v1/media/factorial_waveform.png" style="width: 100%">
</div>
