# MIPS-Compatible CPU
An implementation of a working synthesisable MIPS I ISA compatible CPU.

# Project Overview
The goal of this coursework was to develop a working, synthesizable, MIPS-compatible CPU that interfaced with the world using a memory-mapped bus. The emphasis was on producing a production-quality CPU with a robust testing process that can be sold and distributed to multiple clients, rather than a single circuit working in a single piece of hardware. 

Further information regarding testing and specifications can be found on the [datasheet](https://github.com/IshaanReni/MIPS_CPU/blob/main/Docs/mips_data_sheet.pdf).

# Components
- MIPS CPU Bus: This block is the main interface between the RAM and the rest of the CPU. It adheres to Avalon(R) specifications.
- Control Unit: This block handles the timings of the CPU and transfers data to and from the other blocks. This block is the only block to directly interface with the CPU bus.
- State Machine: This block controls the state the CPU is in - this block also enables the ability to stall, halt, and reset the CPU.
- Instruction Register: This block is responsible for decoding the instruction and sending information to the Control Unit about the current Instruction Word.
- Program Counter: This block keeps track of the address in the RAM of which instruction is currently being executed; setting this to 0 causes the CPU to halt.
- Arithmetic Logic Unit: This block is responsible for all the arithmetic capabilities of the CPU, as well as comparator operations required for conditional branches.
- Load and Store: This block allows writing to the RAM as well as storing data from the RAM into the registers.
- Register File: This block contains all 32 MIPS registers, and has the ability to read from two registers at any point as well as to write data to any one register.

# Design Choices
- Instruction Set: The targetted instruction set is 32-bit big-endian MIPS-I.

- CPU Interface: This CPU used a memory bus-based interface, to be compatible with industrial IP blocks. Instructions and data were fetched over the same interface as the CPU acted as a bus controller to issue read/write transactions to modify memory contents. The CPU had an Avalon compatible memory-mapped interface.

- Avalon Protocol: Avalon is a clock synchronous protocol, where 'readdata' will not become available until the cycle following the read request. The signal 'waitrequest' was used to indicate a stall cycle, which meant that the read or write request could not be completed in the current cycle, and so must have been continued in the next cycle. 

- Reset Behaviour: This CPU did not initiate any memory transactions while the reset signal is high, as the memory may have also been resetting. The effect of resetting was that all the ISA-visible MIPS data registers (including the PC) were set to zero and the following instruction to be executed after the reset was at the address 0xBFC00000 (conventional reset vector for a "real" MIPS CPU).

- Halt Behaviour: The CPU was expected to halt when it executed the instruction at address 0 for testing purposes to see how the memory had been modified.


# Directory Structure
- `Docs` contains the CPU datasheet and files that were included in it.
- `rtl` contains the verilog files that make up the CPU and some higher level testbenches.
- `test` contains everything related to testing including the the automated test script and test cases. 
- `utils` contains the handwritten assembler to convert MIPS to Hex.

# Authors
A working synthesisable MIPS-compatible CPU by Ishaan Reni, Shaheen Amin, Robert Hoppe, Marco Lau, Kia Popat, Conan Quinlivan. December 2021.
