## Project Overview
This project involves designing, simulating, and implementing a matrix-vector multiplication system on an FPGA. The design consists of an array of Multiply-Accumulate (MAC) units that process 8-bit inputs from FIFOs and produce 24-bit outputs. The project is divided into two parts: 
1. **Part 1**: Simulation and timing analysis of the design.
2. **Part 2**: On-board testing using LEDs, 7-segment displays, and SignalTap for debugging.

The goal is to validate the design at multiple stages, from functional simulation to hardware implementation, while ensuring it meets timing constraints.

## Skills Highlighted
- **Verilog HDL**: Designed and implemented a matrix-vector multiplication system using Verilog.
- **Digital Design**: Created an array of MAC units and integrated them with FIFOs for data input.
- **Simulation and Debugging**: Developed testbenches for functional and timing simulation using QuestaSim.
- **FPGA Implementation**: Synthesized the design and implemented it on an FPGA, ensuring it meets timing constraints.
- **Timing Analysis**: Used Synopsys Design Constraints (SDC) to analyze and fix timing issues for a 200 MHz clock.
- **Hardware Debugging**: Utilized SignalTap for in-circuit debugging and verified the Avalon MM interface.
- **System Integration**: Integrated the design with on-board peripherals (LEDs, 7-segment displays, and switches) for real-world testing.

## Project Components
1. **Matrix-Vector Multiplication Design**:
   - Designed an array of 8 MAC units to process 8-bit inputs from FIFOs.
   - Implemented control logic to propagate enable (`En`) and clear (`Cir`) signals through the MAC array.
   - Integrated FIFOs to buffer input data for the MAC units.

2. **Memory Interface**:
   - Designed a module to fetch data from a memory module using the Intel Avalon MM slave interface.
   - Populated FIFOs with data from memory for matrix-vector multiplication.

3. **Testbench**:
   - Created a testbench to simulate the design and verify functionality.
   - Printed interface signals, states, and output signals for debugging purposes.

4. **Timing Analysis**:
   - Modified the clock period in the SDC file to achieve a 200 MHz clock.
   - Analyzed timing reports and fixed design issues to meet timing constraints.

5. **On-Board Testing**:
   - Used LEDs to display the state of the top-level state machine.
   - Displayed MAC outputs on the 7-segment display, controlled by switches.
   - Verified the Avalon MM interface using SignalTap.

## How to Run the Project
1. **Simulation**:
   - Open the Verilog files in QuestaSim.
   - Run the testbench using the following command:
     ```bash
     vsim work.<your_testbench_name> tb -l C:/intelFPGA_lite/21.1/questa_fse/intel/verilog/altera_mf -voptargs="+acc"
     ```
   - Analyze the simulation logs and waveforms to verify functionality.

2. **Synthesis and Timing Analysis**:
   - Open the project in Quartus Prime.
   - Modify the clock period in the SDC file to 5 ns (200 MHz).
   - Synthesize the design and check the timing analyzer report.
   - Fix any timing violations and re-synthesize.

3. **FPGA Implementation**:
   - Program the FPGA with the synthesized design.
   - Use switches to control the 7-segment display and LEDs for state tracking.
   - Use SignalTap to debug the Avalon MM interface and verify correct data transfer.
