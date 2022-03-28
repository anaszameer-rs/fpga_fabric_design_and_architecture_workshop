
# FPGA - Fabric, Design and Architecture

This was a five day workshop that covered 5 modules. The first module focused on running a counter design on Xilinx Vivado and programming it on the Basys3. Demonstrated the area, timing analysis and post implementation simulation. Second module covered the OpenFPGA framework and demonstrated the VTR tool flow with counter design and EArch architecture. Third module covered RISC-V processor core RVMyth. Simulated the core on Vivado and program it on a Basys3 board. Fourth module covered introduction to SOFA and running counter design on SOFA and generate area, timing analysis and post implementation simulation. Fifth module covered the RISC-V processor core RVMyth, taking it through SOFA and generate area, timing analysis and post implementation simulation.

## Day 1: FPGA Introduction and Counter Design Implementation on Basys3 using Vivado?
On the first day of the workshop, a basic introduction to the FPGA was given. A brief history of programmable devices was discussed. Differentiated between ASIC and FPGA flow. Introduced to the FPGA Architecture.Discussed FPGA Programming (local and remote) using Basys3 board and Vivado. Ran a complete flow from RTL design to analysis using Vivado.
### Part 1: FPGA Introduction

1. History of programmable devices
* PLA - Programmable Logic Array consists of a programmable AND gate array and a programmable OR gate array to implement boolean functions
* PAL - Programmable Logic Array consists of a programmable AND gate array and a fix OR gate array to implement a boolean functions
* CPLD - Complex Programmable Logic Device consists of many PLDs and linked by programmable interconnects to implement complex designs
* FPGA - Contains LUTs, FFs, carry chains, and configurable logic blocks to implement complex customizable designs
2. FPGA devices
* Field Programmable Gate Arrays designed to be configured by user
* Field Programmable Gate Arrays contains LUTs, FFs, Specialized blocks including BRAM, DSPs, and configurable logic blocks linked by routing resources to implement complex customizable designs
3. Difference between ASIC and FPGA

ASIC	| 	FPGA
---	| 	---
Designed for specific applications | Can be reconfigured with different designs
Complex and long design process | Relatively Simple design process
Design is pre-verified and then fabricated on silicon | Can be Used for prototyping and validating a design
Consumes less power than FPGA devices| Consumes more power due to standard array logic and most of the time resources might remain unused


4. FPGA applications
* Edge devices
* Hardware acceleration
* Signal Processing
* Aerospace
* Machine Learning
* Device controllers
5. FPGA architecture
* FPGA contains Arrays of configurable logic blocks, BRAM, DSPs linked with routing resources to implement complex designs
* A typical FPGA contains the following elements:
    * CLB - Implements combinational and sequential logic using LUTs and FFs
    * LUT - Implements a logic function
    * FF - Implements sequential logic
    * Carry Chain and Control - Implements arithemetic operations
    * I/O Tiles - To interface with external devices
    * Routing resources - Switch boxes and interconnects to link CLBs and other elements on FPGA
    * DSP Blocks - Specialized hardware for DSP applications
    * BRAM - Block RAM for on-chip RAM support
6. FPGA design methodology
* A typical FPGA design flow consists of the following steps:
    * Architecture description - Define hardware specification
    * Write RTL design and testbench
    * Perform behavioural simulation - simulate the design
    * Sythesis - Synthesize the design by giving constraint file
    * Implementation - Place and Route the design on target fabric
    * Generate the bitstream
    * Program the target FPGA device

### Part 2: Vivado-Counter

4 bit Counter verilog code used to run Vivado synthesis flow on Basys3 Board
```
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

// Description: 4 bit counter with source clock (100MHz) division.

/*

////////////4 bit counter block///////////////////
always @(posedge clk)
begin

if(rst)
begin
counter_out<=4'b0000;
//div_clk <= 1'b0;
end
else
begin
counter_out<= counter_out+1;
end
end

endmodule 



*/

//////////////////////////////////////////////////////////////////////////////////
module counter_clk_div(clk,rst,counter_out);
input clk,rst;
reg div_clk;
reg [25:0] delay_count;
output reg [3:0] counter_out;

//////////clock division block////////////////////


always @(posedge clk)
begin

if(rst)
begin
delay_count<=26'd0;
counter_out<=4'b0000;
div_clk <= 1'b0; //initialise div_clk
counter_out<=4'b0000;

//uncomment this line while running just the div clock counter for simulation purpose
//counter_out<=4'b0000;
end
else

//uncomment this line while running just the div clock counter for simulation purpose
if(delay_count==26'd212)

//comment this line while running just the div clock counter for simulation purpose
//if(delay_count==26'd32112212)
begin
delay_count<=26'd0; //reset upon reaching the max value
div_clk <= ~div_clk;  //generating a slow clock
end
else
begin
delay_count<=delay_count+1;
end
end


/////////////4 bit counter block///////////////////
always @(posedge div_clk)
begin

if(rst)
begin
counter_out<=4'b0000;
end
else
begin
counter_out<= counter_out+1;
end
end

endmodule 
```
Testbench used to verify 4-bit counter design
```
`timescale 1ns / 1ps

module test_counter();
reg clk, reset;
wire [3:0] out;

//create an instance of the design
counter_clk_div dut(clk, reset, out);  

initial begin

//note that these statements are sequential.. execute one after the other 

//$dumpfile ("count.vcd"); 
//$dumpvars(0,upcounter_testbench);

clk=0;  //at time=0

reset=1;//at time=0

#20; //delay 20 units
reset=0; //after 20 units of time, reset becomes 0


end


always 
#5 clk=~clk;  // toggle or negate the clk input every 5 units of time


endmodule 

```
1. Vivado Simulation Elaboration
* Simulated the 4 bit counter by adding counter RTL in verilog sources and testbench in simulation sources 

![sim](https://user-images.githubusercontent.com/92034288/160283570-95f5e65b-2992-4b68-846b-eb7dc1d1262b.png)
 *Figure 1. Simulation of a 4-bit counter - counter_clk_div* 
   
 ![sch](https://user-images.githubusercontent.com/92034288/160284097-adf0aae9-31e2-4030-b226-e1addf4e6892.png) 
  *Figure 2. Schematic of a 4-bit counter - counter_clk_div*

2. Pin Mapping and sdc constraint file
* Provide pin constraints for mapping in/out of counter on specific pins
* Set clock frequency from constraint wizard and run synthesis

![constraint](https://user-images.githubusercontent.com/92034288/160284313-a30c6cac-ae69-4dc3-9eca-c2337abde173.png)
*Figure 3. Snapshot of Constraints for a 4-bit counter - counter_clk_div*

3. Synthesis in Vivado
* Run synthesis to synthesize the design for Basys3
* Implement the design on Basys3
4. Area, Power and slack
* Generate analysis reports by selecting generate util, generate power and generate timing summary respectively

![power](https://user-images.githubusercontent.com/92034288/160284817-daef5f4a-8261-4b60-bc91-3bdfc649ea75.png)
*Figure 4. Snapshot of power analysis of 4-bit counter design*

![util](https://user-images.githubusercontent.com/92034288/160284821-8da1402e-e81b-4134-9ada-00cd849c2bf7.png)
*Figure 5. Snapshot of resource usage of 4-bit counter design*

5. Generate bitstream to program Basys3 locally by running generate bitstream and program FPGA via JTAG interface

### Part 3: VIO-Counter

  * VIO is a virtual interface used for virtually driving and monitoring the FPGA signals 
  * To run the counter via VIO, first modify the design.
  * Create a VIO instance using template
  * Copy VIO instance in the counter code, rename the VIO signals for connections
  * Run analysis
  * Run synthesis and
  * Generate bitstream and program the board
  * Open VIO window in hardware manager, add probes and observe signals.

## Day 2: OpenFPGA Flow

### Part 1: OpenFPGA Intro
* openFPGA is an open source frame work that involves open source tools to generate customizable FPGA fabrics according to the desired application and reduces the design cycle immensely.
* openFPGA frame work supports:
  * Fabric generation
  * Testbench generation to test the generated fabric automatically
  * Bitstream generation
* openFPGA architecture is defined by using an XML file
* OpenFPGA architecture consists of CLBs, LUTs, FLEs, Carry-chains, FFs, BRAM, DSPs, I/O tiles, Clock trees, Switch boxes and connection blocks for routing.

### Part 2: VPR

* VPR is a tool used for place and route a design on a given architecture and provides analysis reports.
* The following command used to perform the place and route using VPR
```
 $VTR_ROOT/vpr/vpr \
    $VTR_ROOT/vtr_flow/arch/timing/EArch.xml \
    $VTR_ROOT/vtr_flow/benchmarks/blif/tseng.blif \
    --route_chan_width 100 \
    --analysis --disp on
```
* After running the command VPR CAD Flow opens in GUI
* VPR generates different log files in run directory .net represents packing, .place represent placement of design on fpga fabric and .route represents routing

![Screenshot from 2022-03-26 15-03-26](https://user-images.githubusercontent.com/92034288/160288076-46966d59-01d5-4373-a757-3a03d5f29766.png)

*Figure 7. Snapshot of VPR CAD Flow to Visualize Mapped design*

### Part 3: VTR

*  VTR is a tool used to map a design on a given architecture
*  Synthesis the design using yosys or odin-II
*  Perform optimization using abc
*  Place and route using VPR
*  Run the automated VTR flow using the following command:

```
 $VTR_ROOT/vtr_flow/scripts/run_vtr_flow.py 
 ../fpga_workshop_collaterals/Day2/counter_files/counter.v 
 $VTR_ROOT/vtr_flow/arch/timing/EArch.xml 
 -power -cmos_tech  $VTR_ROOT/vtr_flow/tech/PTM_45nm/45nm.xml 
 -temp_dir . --route_chan_width 100

```

![Screenshot from 2022-03-27 21-14-24](https://user-images.githubusercontent.com/92034288/160416752-16482ba5-596c-46ab-8a20-356e26592302.png)

*Figure 8. Snapshot of VTR flow successful Run and Generated Timing and Log Files*

![fig9](https://user-images.githubusercontent.com/92034288/160433534-ee3894f9-7aed-4d05-9d00-48bff0e65e03.png)

*Figure 9. Power Breakdown Results for Counter*

![fig10](https://user-images.githubusercontent.com/92034288/160433590-1b428a14-f7a4-4b97-ac47-8404987b22c3.png)

*Figure 10. Slack Results for Counter*

![Screenshot from 2022-03-27 21-30-45](https://user-images.githubusercontent.com/92034288/160425846-3087700d-b1c1-4f6d-b372-d7cc76a12e05.png)

*Figure 11. Simulation of post_synthesis Counter Design on Vivado*


## Day 3: Introduction to RISC-V Core Programming on Vivado

* Synthesize and implement rvmyth core using Vivado
* rvmyth is a pipelined processor core which reads stored instructions from memory to add 1st 10 natural numbers 
* rvmyth core verilog code is given below:

```
//_\TLV_version 1d: tl-x.org, generated by SandPiper(TM) 1.11-2021/01/28-beta
//`include "sp_verilog.vh" //_\SV
    // Included URL: "https://raw.githubusercontent.com/shivanishah269/risc-v-core/master/FPGA_Implementation/riscv_shell_lib.tlv"// Included URL: "https://raw.githubusercontent.com/stevehoover/warp-v_includes/2d6d36baa4d2bc62321f982f78c8fe1456641a43/risc-v_defs.tlv"
/*
Copyright (c) 2015, Steven F. Hoover

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * The name of Steven F. Hoover
      may not be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//`include "sp_verilog.vh"

`ifndef SP_DEFAULT
`define SP_DEFAULT
/*
Copyright (c) 2015, Steven F. Hoover

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * The name of Steven F. Hoover
      may not be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


// File included by SandPiper-generated code for the default project configuration.
//`include "sandpiper.vh"

/*
Copyright (c) 2015, Steven F. Hoover

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * The name of Steven F. Hoover
      may not be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Project-independent SandPiper header file.

`ifndef SANDPIPER_VH
`define SANDPIPER_VH


// Note, these have no SP prefix, so collisions are possible.
     

`ifdef WHEN
   // Make sure user definition does not collide.
   !!!ERROR: WHEN macro already defined
`else
   `ifdef SP_PHYS
      // Phys compilation disabled X-injection.
      `define WHEN(valid_sig)
   `else
      // Inject X.
      `define WHEN(valid_sig) !valid_sig ? 'x :
   `endif
`endif


// SandPiper does not generate set/reset flops.  Reset is implemented as combinational
// logic, and it is up to synthesis to infer set/reset flops when possible.
//`ifdef RESET
//   // Make sure user definition does not collide.
//   !!!ERROR: RESET macro already defined
//`else
//   `define RESET(i, reset) ((reset) ? '0 : i)
//`endif
//
//`ifdef SET
//   // Make sure user definition does not collide.
//   !!!ERROR: SET macro already defined
//`else
//   `define SET(i, set) ((set) ? '1 : i)
//`endif

// Since SandPiper required use of all signals, this is useful to create a
// bogus use and keep SandPiper happy when a signal, by intent, has no uses.
`define BOGUS_USE(ignore)

`endif  // SANDPIPER_VH



///////////////


// Latch macros.  Inject 'x in simulation for clk === 'x.

// A-phase latch.
`ifdef SP_PHYS
`define TLV_LATCH(in, out, clk) \
always @ (in, clk) begin        \
  if (clk === 1'b1)             \
    out <= in;                  \
  else if (clk === 1'bx)        \
    out <= 'x;                  \
end
`else
`define TLV_LATCH(in, out, clk) always @ (in, clk) if (clk == 1'b1) out <= in;
`endif  // SP_PHYS

// B-phase latch.
`ifdef SP_PHYS
`define TLV_BLATCH(out, in, clk) \
always @ (in, clk) begin         \
  if (!clk === 1'b1)             \
    out <= in;                   \
  else if (!clk === 1'bx)        \
    out <= 'x;                   \
end
`else
`define TLV_BLATCH(out, in, clk) always @ (in, clk) if (!clk == 1'b1) out <= in;
`endif  // SP_PHYS


	   
`endif  // SP_DEFAULT


///////////////////////////////////////////////////sp_verilog end//


// Clock gate module used by SandPiper default project.

module clk_gate (output gated_clk, input free_clk, func_en, pwr_en, gating_override);
   wire clk_en;
   reg latched_clk_en  /*verilator clock_enable*/;
   assign clk_en = func_en & (pwr_en | gating_override);
   `TLV_BLATCH(latched_clk_en, clk_en, free_clk)
   assign gated_clk = latched_clk_en & free_clk;
endmodule




////////////////////////end clkgate//////////////////   
//for simulation purpose, make out an output port
   module core (input clk, input reset, output [7:0] out);
// Generated by SandPiper(TM) 1.11-2021/01/28-beta from Redwood EDA.
// Redwood EDA does not claim intellectual property rights to this file and provides no warranty regarding its correctness or quality.
reg [7:0] out;

////////////////////////////////////start test_gen/////////////////////////////////////////////////
//`include "sandpiper_gen.vh"


genvar dmem, imem, xreg;


//
// Signals declared top-level.
//

// For |cpu$br_tgt_pc.
wire [31:0] CPU_br_tgt_pc_a2;
reg  [31:0] CPU_br_tgt_pc_a3;

// For |cpu$dec_bits.
wire [10:0] CPU_dec_bits_a1;

// For |cpu$dmem_addr.
wire [3:0] CPU_dmem_addr_a4;

// For |cpu$dmem_rd_data.
wire [31:0] CPU_dmem_rd_data_a4;
reg  [31:0] CPU_dmem_rd_data_a5;

// For |cpu$dmem_rd_en.
wire CPU_dmem_rd_en_a4;

// For |cpu$dmem_wr_data.
wire [31:0] CPU_dmem_wr_data_a4;

// For |cpu$dmem_wr_en.
wire CPU_dmem_wr_en_a4;

// For |cpu$funct3.
wire [2:0] CPU_funct3_a1;

// For |cpu$funct3_valid.
wire CPU_funct3_valid_a1;

// For |cpu$funct7.
wire [6:0] CPU_funct7_a1;

// For |cpu$funct7_valid.
wire CPU_funct7_valid_a1;

// For |cpu$imem_rd_addr.
wire [31:0] CPU_imem_rd_addr_a0;
reg  [31:0] CPU_imem_rd_addr_a1;

// For |cpu$imem_rd_data.
wire [31:0] CPU_imem_rd_data_a1;

// For |cpu$imem_rd_en.
wire CPU_imem_rd_en_a0;
reg  CPU_imem_rd_en_a1;

// For |cpu$imm.
wire [31:0] CPU_imm_a1;
reg  [31:0] CPU_imm_a2,
            CPU_imm_a3;

// For |cpu$inc_pc.
wire [31:0] CPU_inc_pc_a1;
reg  [31:0] CPU_inc_pc_a2,
            CPU_inc_pc_a3;

// For |cpu$instr.
wire [31:0] CPU_instr_a1;

// For |cpu$is_add.
wire CPU_is_add_a1;
reg  CPU_is_add_a2,
     CPU_is_add_a3;

// For |cpu$is_addi.
wire CPU_is_addi_a1;
reg  CPU_is_addi_a2,
     CPU_is_addi_a3;

// For |cpu$is_and.
wire CPU_is_and_a1;
reg  CPU_is_and_a2,
     CPU_is_and_a3;

// For |cpu$is_andi.
wire CPU_is_andi_a1;
reg  CPU_is_andi_a2,
     CPU_is_andi_a3;

// For |cpu$is_auipc.
wire CPU_is_auipc_a1;
reg  CPU_is_auipc_a2,
     CPU_is_auipc_a3;

// For |cpu$is_b_instr.
wire CPU_is_b_instr_a1;

// For |cpu$is_beq.
wire CPU_is_beq_a1;
reg  CPU_is_beq_a2,
     CPU_is_beq_a3;

// For |cpu$is_bge.
wire CPU_is_bge_a1;
reg  CPU_is_bge_a2,
     CPU_is_bge_a3;

// For |cpu$is_bgeu.
wire CPU_is_bgeu_a1;
reg  CPU_is_bgeu_a2,
     CPU_is_bgeu_a3;

// For |cpu$is_blt.
wire CPU_is_blt_a1;
reg  CPU_is_blt_a2,
     CPU_is_blt_a3;

// For |cpu$is_bltu.
wire CPU_is_bltu_a1;
reg  CPU_is_bltu_a2,
     CPU_is_bltu_a3;

// For |cpu$is_bne.
wire CPU_is_bne_a1;
reg  CPU_is_bne_a2,
     CPU_is_bne_a3;

// For |cpu$is_i_instr.
wire CPU_is_i_instr_a1;

// For |cpu$is_j_instr.
wire CPU_is_j_instr_a1;

// For |cpu$is_jal.
wire CPU_is_jal_a1;
reg  CPU_is_jal_a2,
     CPU_is_jal_a3;

// For |cpu$is_jalr.
wire CPU_is_jalr_a1;
reg  CPU_is_jalr_a2,
     CPU_is_jalr_a3;

// For |cpu$is_jump.
wire CPU_is_jump_a1;
reg  CPU_is_jump_a2,
     CPU_is_jump_a3;

// For |cpu$is_load.
wire CPU_is_load_a1;
reg  CPU_is_load_a2,
     CPU_is_load_a3;

// For |cpu$is_lui.
wire CPU_is_lui_a1;
reg  CPU_is_lui_a2,
     CPU_is_lui_a3;

// For |cpu$is_or.
wire CPU_is_or_a1;
reg  CPU_is_or_a2,
     CPU_is_or_a3;

// For |cpu$is_ori.
wire CPU_is_ori_a1;
reg  CPU_is_ori_a2,
     CPU_is_ori_a3;

// For |cpu$is_r_instr.
wire CPU_is_r_instr_a1;

// For |cpu$is_s_instr.
wire CPU_is_s_instr_a1;
reg  CPU_is_s_instr_a2,
     CPU_is_s_instr_a3,
     CPU_is_s_instr_a4;

// For |cpu$is_sb.
wire CPU_is_sb_a1;

// For |cpu$is_sh.
wire CPU_is_sh_a1;

// For |cpu$is_sll.
wire CPU_is_sll_a1;
reg  CPU_is_sll_a2,
     CPU_is_sll_a3;

// For |cpu$is_slli.
wire CPU_is_slli_a1;
reg  CPU_is_slli_a2,
     CPU_is_slli_a3;

// For |cpu$is_slt.
wire CPU_is_slt_a1;
reg  CPU_is_slt_a2,
     CPU_is_slt_a3;

// For |cpu$is_slti.
wire CPU_is_slti_a1;
reg  CPU_is_slti_a2,
     CPU_is_slti_a3;

// For |cpu$is_sltiu.
wire CPU_is_sltiu_a1;
reg  CPU_is_sltiu_a2,
     CPU_is_sltiu_a3;

// For |cpu$is_sltu.
wire CPU_is_sltu_a1;
reg  CPU_is_sltu_a2,
     CPU_is_sltu_a3;

// For |cpu$is_sra.
wire CPU_is_sra_a1;
reg  CPU_is_sra_a2,
     CPU_is_sra_a3;

// For |cpu$is_srai.
wire CPU_is_srai_a1;
reg  CPU_is_srai_a2,
     CPU_is_srai_a3;

// For |cpu$is_srl.
wire CPU_is_srl_a1;
reg  CPU_is_srl_a2,
     CPU_is_srl_a3;

// For |cpu$is_srli.
wire CPU_is_srli_a1;
reg  CPU_is_srli_a2,
     CPU_is_srli_a3;

// For |cpu$is_sub.
wire CPU_is_sub_a1;
reg  CPU_is_sub_a2,
     CPU_is_sub_a3;

// For |cpu$is_sw.
wire CPU_is_sw_a1;

// For |cpu$is_u_instr.
wire CPU_is_u_instr_a1;

// For |cpu$is_xor.
wire CPU_is_xor_a1;
reg  CPU_is_xor_a2,
     CPU_is_xor_a3;

// For |cpu$is_xori.
wire CPU_is_xori_a1;
reg  CPU_is_xori_a2,
     CPU_is_xori_a3;

// For |cpu$jalr_tgt_pc.
wire [31:0] CPU_jalr_tgt_pc_a2;
reg  [31:0] CPU_jalr_tgt_pc_a3;

// For |cpu$ld_data.
wire [31:0] CPU_ld_data_a5;

// For |cpu$opcode.
wire [6:0] CPU_opcode_a1;

// For |cpu$pc.
wire [31:0] CPU_pc_a0;
reg  [31:0] CPU_pc_a1,
            CPU_pc_a2,
            CPU_pc_a3;

// For |cpu$rd.
wire [4:0] CPU_rd_a1;
reg  [4:0] CPU_rd_a2,
           CPU_rd_a3,
           CPU_rd_a4,
           CPU_rd_a5;

// For |cpu$rd_valid.
wire CPU_rd_valid_a1;
reg  CPU_rd_valid_a2,
     CPU_rd_valid_a3,
     CPU_rd_valid_a4;

// For |cpu$reset.
wire CPU_reset_a0;
reg  CPU_reset_a1,
     CPU_reset_a2,
     CPU_reset_a3,
     CPU_reset_a4;

// For |cpu$result.
wire [31:0] CPU_result_a3;
reg  [5:2] CPU_result_a4;

// For |cpu$rf_rd_data1.
wire [31:0] CPU_rf_rd_data1_a2;

// For |cpu$rf_rd_data2.
wire [31:0] CPU_rf_rd_data2_a2;

// For |cpu$rf_rd_en1.
wire CPU_rf_rd_en1_a2;

// For |cpu$rf_rd_en2.
wire CPU_rf_rd_en2_a2;

// For |cpu$rf_rd_index1.
wire [4:0] CPU_rf_rd_index1_a2;

// For |cpu$rf_rd_index2.
wire [4:0] CPU_rf_rd_index2_a2;

// For |cpu$rf_wr_data.
wire [31:0] CPU_rf_wr_data_a3;

// For |cpu$rf_wr_en.
wire CPU_rf_wr_en_a3;

// For |cpu$rf_wr_index.
wire [4:0] CPU_rf_wr_index_a3;

// For |cpu$rs1.
wire [4:0] CPU_rs1_a1;
reg  [4:0] CPU_rs1_a2;

// For |cpu$rs1_valid.
wire CPU_rs1_valid_a1;
reg  CPU_rs1_valid_a2;

// For |cpu$rs2.
wire [4:0] CPU_rs2_a1;
reg  [4:0] CPU_rs2_a2;

// For |cpu$rs2_valid.
wire CPU_rs2_valid_a1;
reg  CPU_rs2_valid_a2;

// For |cpu$sltiu_result.
wire CPU_sltiu_result_a3;

// For |cpu$sltu_result.
wire CPU_sltu_result_a3;

// For |cpu$src1_value.
wire [31:0] CPU_src1_value_a2;
reg  [31:0] CPU_src1_value_a3;

// For |cpu$src2_value.
wire [31:0] CPU_src2_value_a2;
reg  [31:0] CPU_src2_value_a3,
            CPU_src2_value_a4;

// For |cpu$taken_br.
wire CPU_taken_br_a3;

// For |cpu$valid.
wire CPU_valid_a3;
reg  CPU_valid_a4;

// For |cpu$valid_jump.
wire CPU_valid_jump_a3;
reg  CPU_valid_jump_a4,
     CPU_valid_jump_a5;

// For |cpu$valid_load.
wire CPU_valid_load_a3;
reg  CPU_valid_load_a4,
     CPU_valid_load_a5;

// For |cpu$valid_taken_br.
wire CPU_valid_taken_br_a3;
reg  CPU_valid_taken_br_a4,
     CPU_valid_taken_br_a5;

// For |cpu/dmem$value.
wire [31:0] CPU_Dmem_value_a4 [15:0];
reg  [31:0] CPU_Dmem_value_a5 [15:0];

// For |cpu/imem$instr.
wire [31:0] CPU_Imem_instr_a1 [9:0];

// For |cpu/xreg$value.
wire [31:0] CPU_Xreg_value_a3 [31:0];
reg  [31:0] CPU_Xreg_value_a4 [31:0],
            CPU_Xreg_value_a5 [31:0];


//
// Scope: |cpu
//

// Clock signals.
wire clkP_CPU_dmem_rd_en_a5 ;
wire clkP_CPU_rd_valid_a2 ;
wire clkP_CPU_rd_valid_a3 ;
wire clkP_CPU_rd_valid_a4 ;
wire clkP_CPU_rd_valid_a5 ;
wire clkP_CPU_rs1_valid_a2 ;
wire clkP_CPU_rs2_valid_a2 ;


generate


   //
   // Scope: |cpu
   //

      // For $br_tgt_pc.
      always @(posedge clk) CPU_br_tgt_pc_a3[31:0] <= CPU_br_tgt_pc_a2[31:0];

      // For $dmem_rd_data.
      always @(posedge clkP_CPU_dmem_rd_en_a5) CPU_dmem_rd_data_a5[31:0] <= CPU_dmem_rd_data_a4[31:0];

      // For $imem_rd_addr.
      always @(posedge clk) CPU_imem_rd_addr_a1[31:0] <= CPU_imem_rd_addr_a0[31:0];

      // For $imem_rd_en.
      always @(posedge clk) CPU_imem_rd_en_a1 <= CPU_imem_rd_en_a0;

      // For $imm.
      always @(posedge clk) CPU_imm_a2[31:0] <= CPU_imm_a1[31:0];
      always @(posedge clk) CPU_imm_a3[31:0] <= CPU_imm_a2[31:0];

      // For $inc_pc.
      always @(posedge clk) CPU_inc_pc_a2[31:0] <= CPU_inc_pc_a1[31:0];
      always @(posedge clk) CPU_inc_pc_a3[31:0] <= CPU_inc_pc_a2[31:0];

      // For $is_add.
      always @(posedge clk) CPU_is_add_a2 <= CPU_is_add_a1;
      always @(posedge clk) CPU_is_add_a3 <= CPU_is_add_a2;

      // For $is_addi.
      always @(posedge clk) CPU_is_addi_a2 <= CPU_is_addi_a1;
      always @(posedge clk) CPU_is_addi_a3 <= CPU_is_addi_a2;

      // For $is_and.
      always @(posedge clk) CPU_is_and_a2 <= CPU_is_and_a1;
      always @(posedge clk) CPU_is_and_a3 <= CPU_is_and_a2;

      // For $is_andi.
      always @(posedge clk) CPU_is_andi_a2 <= CPU_is_andi_a1;
      always @(posedge clk) CPU_is_andi_a3 <= CPU_is_andi_a2;

      // For $is_auipc.
      always @(posedge clk) CPU_is_auipc_a2 <= CPU_is_auipc_a1;
      always @(posedge clk) CPU_is_auipc_a3 <= CPU_is_auipc_a2;

      // For $is_beq.
      always @(posedge clk) CPU_is_beq_a2 <= CPU_is_beq_a1;
      always @(posedge clk) CPU_is_beq_a3 <= CPU_is_beq_a2;

      // For $is_bge.
      always @(posedge clk) CPU_is_bge_a2 <= CPU_is_bge_a1;
      always @(posedge clk) CPU_is_bge_a3 <= CPU_is_bge_a2;

      // For $is_bgeu.
      always @(posedge clk) CPU_is_bgeu_a2 <= CPU_is_bgeu_a1;
      always @(posedge clk) CPU_is_bgeu_a3 <= CPU_is_bgeu_a2;

      // For $is_blt.
      always @(posedge clk) CPU_is_blt_a2 <= CPU_is_blt_a1;
      always @(posedge clk) CPU_is_blt_a3 <= CPU_is_blt_a2;

      // For $is_bltu.
      always @(posedge clk) CPU_is_bltu_a2 <= CPU_is_bltu_a1;
      always @(posedge clk) CPU_is_bltu_a3 <= CPU_is_bltu_a2;

      // For $is_bne.
      always @(posedge clk) CPU_is_bne_a2 <= CPU_is_bne_a1;
      always @(posedge clk) CPU_is_bne_a3 <= CPU_is_bne_a2;

      // For $is_jal.
      always @(posedge clk) CPU_is_jal_a2 <= CPU_is_jal_a1;
      always @(posedge clk) CPU_is_jal_a3 <= CPU_is_jal_a2;

      // For $is_jalr.
      always @(posedge clk) CPU_is_jalr_a2 <= CPU_is_jalr_a1;
      always @(posedge clk) CPU_is_jalr_a3 <= CPU_is_jalr_a2;

      // For $is_jump.
      always @(posedge clk) CPU_is_jump_a2 <= CPU_is_jump_a1;
      always @(posedge clk) CPU_is_jump_a3 <= CPU_is_jump_a2;

      // For $is_load.
      always @(posedge clk) CPU_is_load_a2 <= CPU_is_load_a1;
      always @(posedge clk) CPU_is_load_a3 <= CPU_is_load_a2;

      // For $is_lui.
      always @(posedge clk) CPU_is_lui_a2 <= CPU_is_lui_a1;
      always @(posedge clk) CPU_is_lui_a3 <= CPU_is_lui_a2;

      // For $is_or.
      always @(posedge clk) CPU_is_or_a2 <= CPU_is_or_a1;
      always @(posedge clk) CPU_is_or_a3 <= CPU_is_or_a2;

      // For $is_ori.
      always @(posedge clk) CPU_is_ori_a2 <= CPU_is_ori_a1;
      always @(posedge clk) CPU_is_ori_a3 <= CPU_is_ori_a2;

      // For $is_s_instr.
      always @(posedge clk) CPU_is_s_instr_a2 <= CPU_is_s_instr_a1;
      always @(posedge clk) CPU_is_s_instr_a3 <= CPU_is_s_instr_a2;
      always @(posedge clk) CPU_is_s_instr_a4 <= CPU_is_s_instr_a3;

      // For $is_sll.
      always @(posedge clk) CPU_is_sll_a2 <= CPU_is_sll_a1;
      always @(posedge clk) CPU_is_sll_a3 <= CPU_is_sll_a2;

      // For $is_slli.
      always @(posedge clk) CPU_is_slli_a2 <= CPU_is_slli_a1;
      always @(posedge clk) CPU_is_slli_a3 <= CPU_is_slli_a2;

      // For $is_slt.
      always @(posedge clk) CPU_is_slt_a2 <= CPU_is_slt_a1;
      always @(posedge clk) CPU_is_slt_a3 <= CPU_is_slt_a2;

      // For $is_slti.
      always @(posedge clk) CPU_is_slti_a2 <= CPU_is_slti_a1;
      always @(posedge clk) CPU_is_slti_a3 <= CPU_is_slti_a2;

      // For $is_sltiu.
      always @(posedge clk) CPU_is_sltiu_a2 <= CPU_is_sltiu_a1;
      always @(posedge clk) CPU_is_sltiu_a3 <= CPU_is_sltiu_a2;

      // For $is_sltu.
      always @(posedge clk) CPU_is_sltu_a2 <= CPU_is_sltu_a1;
      always @(posedge clk) CPU_is_sltu_a3 <= CPU_is_sltu_a2;

      // For $is_sra.
      always @(posedge clk) CPU_is_sra_a2 <= CPU_is_sra_a1;
      always @(posedge clk) CPU_is_sra_a3 <= CPU_is_sra_a2;

      // For $is_srai.
      always @(posedge clk) CPU_is_srai_a2 <= CPU_is_srai_a1;
      always @(posedge clk) CPU_is_srai_a3 <= CPU_is_srai_a2;

      // For $is_srl.
      always @(posedge clk) CPU_is_srl_a2 <= CPU_is_srl_a1;
      always @(posedge clk) CPU_is_srl_a3 <= CPU_is_srl_a2;

      // For $is_srli.
      always @(posedge clk) CPU_is_srli_a2 <= CPU_is_srli_a1;
      always @(posedge clk) CPU_is_srli_a3 <= CPU_is_srli_a2;

      // For $is_sub.
      always @(posedge clk) CPU_is_sub_a2 <= CPU_is_sub_a1;
      always @(posedge clk) CPU_is_sub_a3 <= CPU_is_sub_a2;

      // For $is_xor.
      always @(posedge clk) CPU_is_xor_a2 <= CPU_is_xor_a1;
      always @(posedge clk) CPU_is_xor_a3 <= CPU_is_xor_a2;

      // For $is_xori.
      always @(posedge clk) CPU_is_xori_a2 <= CPU_is_xori_a1;
      always @(posedge clk) CPU_is_xori_a3 <= CPU_is_xori_a2;

      // For $jalr_tgt_pc.
      always @(posedge clk) CPU_jalr_tgt_pc_a3[31:0] <= CPU_jalr_tgt_pc_a2[31:0];

      // For $pc.
      always @(posedge clk) CPU_pc_a1[31:0] <= CPU_pc_a0[31:0];
      always @(posedge clk) CPU_pc_a2[31:0] <= CPU_pc_a1[31:0];
      always @(posedge clk) CPU_pc_a3[31:0] <= CPU_pc_a2[31:0];

      // For $rd.
      always @(posedge clkP_CPU_rd_valid_a2) CPU_rd_a2[4:0] <= CPU_rd_a1[4:0];
      always @(posedge clkP_CPU_rd_valid_a3) CPU_rd_a3[4:0] <= CPU_rd_a2[4:0];
      always @(posedge clkP_CPU_rd_valid_a4) CPU_rd_a4[4:0] <= CPU_rd_a3[4:0];
      always @(posedge clkP_CPU_rd_valid_a5) CPU_rd_a5[4:0] <= CPU_rd_a4[4:0];

      // For $rd_valid.
      always @(posedge clk) CPU_rd_valid_a2 <= CPU_rd_valid_a1;
      always @(posedge clk) CPU_rd_valid_a3 <= CPU_rd_valid_a2;
      always @(posedge clk) CPU_rd_valid_a4 <= CPU_rd_valid_a3;

      // For $reset.
      always @(posedge clk) CPU_reset_a1 <= CPU_reset_a0;
      always @(posedge clk) CPU_reset_a2 <= CPU_reset_a1;
      always @(posedge clk) CPU_reset_a3 <= CPU_reset_a2;
      always @(posedge clk) CPU_reset_a4 <= CPU_reset_a3;

      // For $result.
      always @(posedge clk) CPU_result_a4[5:2] <= CPU_result_a3[5:2];

      // For $rs1.
      always @(posedge clkP_CPU_rs1_valid_a2) CPU_rs1_a2[4:0] <= CPU_rs1_a1[4:0];

      // For $rs1_valid.
      always @(posedge clk) CPU_rs1_valid_a2 <= CPU_rs1_valid_a1;

      // For $rs2.
      always @(posedge clkP_CPU_rs2_valid_a2) CPU_rs2_a2[4:0] <= CPU_rs2_a1[4:0];

      // For $rs2_valid.
      always @(posedge clk) CPU_rs2_valid_a2 <= CPU_rs2_valid_a1;

      // For $src1_value.
      always @(posedge clk) CPU_src1_value_a3[31:0] <= CPU_src1_value_a2[31:0];

      // For $src2_value.
      always @(posedge clk) CPU_src2_value_a3[31:0] <= CPU_src2_value_a2[31:0];
      always @(posedge clk) CPU_src2_value_a4[31:0] <= CPU_src2_value_a3[31:0];

      // For $valid.
      always @(posedge clk) CPU_valid_a4 <= CPU_valid_a3;

      // For $valid_jump.
      always @(posedge clk) CPU_valid_jump_a4 <= CPU_valid_jump_a3;
      always @(posedge clk) CPU_valid_jump_a5 <= CPU_valid_jump_a4;

      // For $valid_load.
      always @(posedge clk) CPU_valid_load_a4 <= CPU_valid_load_a3;
      always @(posedge clk) CPU_valid_load_a5 <= CPU_valid_load_a4;

      // For $valid_taken_br.
      always @(posedge clk) CPU_valid_taken_br_a4 <= CPU_valid_taken_br_a3;
      always @(posedge clk) CPU_valid_taken_br_a5 <= CPU_valid_taken_br_a4;


      //
      // Scope: /dmem[15:0]
      //
      for (dmem = 0; dmem <= 15; dmem=dmem+1) begin : L1gen_CPU_Dmem
         // For $value.
         always @(posedge clk) CPU_Dmem_value_a5[dmem][31:0] <= CPU_Dmem_value_a4[dmem][31:0];

      end

      //
      // Scope: /xreg[31:0]
      //
      for (xreg = 0; xreg <= 31; xreg=xreg+1) begin : L1gen_CPU_Xreg
         // For $value.
         always @(posedge clk) CPU_Xreg_value_a4[xreg][31:0] <= CPU_Xreg_value_a3[xreg][31:0];
         always @(posedge clk) CPU_Xreg_value_a5[xreg][31:0] <= CPU_Xreg_value_a4[xreg][31:0];

      end



endgenerate



//
// Gated clocks.
//

generate



   //
   // Scope: |cpu
   //

      clk_gate gen_clkP_CPU_dmem_rd_en_a5(clkP_CPU_dmem_rd_en_a5, clk, 1'b1, CPU_dmem_rd_en_a4, 1'b0);
      clk_gate gen_clkP_CPU_rd_valid_a2(clkP_CPU_rd_valid_a2, clk, 1'b1, CPU_rd_valid_a1, 1'b0);
      clk_gate gen_clkP_CPU_rd_valid_a3(clkP_CPU_rd_valid_a3, clk, 1'b1, CPU_rd_valid_a2, 1'b0);
      clk_gate gen_clkP_CPU_rd_valid_a4(clkP_CPU_rd_valid_a4, clk, 1'b1, CPU_rd_valid_a3, 1'b0);
      clk_gate gen_clkP_CPU_rd_valid_a5(clkP_CPU_rd_valid_a5, clk, 1'b1, CPU_rd_valid_a4, 1'b0);
      clk_gate gen_clkP_CPU_rs1_valid_a2(clkP_CPU_rs1_valid_a2, clk, 1'b1, CPU_rs1_valid_a1, 1'b0);
      clk_gate gen_clkP_CPU_rs2_valid_a2(clkP_CPU_rs2_valid_a2, clk, 1'b1, CPU_rs2_valid_a1, 1'b0);



endgenerate


////////////////////////////////////end test_gen///////////////////////////////////////////////////////
generate //_\TLV
// /====================\
   // | Sum 1 to 9 Program |
   // \====================/
   //
   // Program for MYTH Workshop to test RV32I
   // Add 1,2,3,...,9 (in that order).
   //
   // Regs:
   //  r10 (a0): In: 0, Out: final sum
   //  r12 (a2): 10
   //  r13 (a3): 1..10
   //  r14 (a4): Sum
   // 
   // External to function:
   // Inst #0: ADD,r10,r0,r0             // Initialize r10 (a0) to 0.
   // Function:
   // Inst #1: ADD,r14,r10,r0            // Initialize sum register a4 with 0x0
   // Inst #2: ADDI,r12,r10,1010         // Store count of 10 in register a2.
   // Inst #3: ADD,r13,r10,r0            // Initialize intermediate sum register a3 with 0
   // Loop:
   // Inst #4: ADD,r14,r13,r14           // Incremental addition
   // Inst #5: ADDI,r13,r13,1            // Increment intermediate register by 1
   // Inst #6: BLT,r13,r12,1111111111000 // If a3 is less than a2, branch to label named <loop>
   // Inst #7: ADD,r10,r14,r0            // Store final result to register a0 so that it can be read by main program
   // Inst #8: SW,r0,r10,10000           // Store the value of r10 into address 17.
   // Inst #9: LW,r17,r0,10000           // Load the value from 
   
   // Optional:
   // m4_asm(JAL, r7, 00000000000000000000) // Done. Jump to itself (infinite loop). (Up to 20-bit signed immediate plus implicit 0 bit (unlike JALR) provides byte address; last immediate bit should also be 0)
   

   //_|cpu
      //_@0
         assign CPU_reset_a0 = reset;
      
      //Fetch
         // Next PC
         assign CPU_pc_a0[31:0] = (CPU_reset_a1) ? 32'b0 : 
                     (CPU_valid_taken_br_a3) ? CPU_br_tgt_pc_a3 : 
                     (CPU_valid_load_a3) ? CPU_inc_pc_a3 : 
                     (CPU_valid_jump_a3 && CPU_is_jal_a3) ? CPU_br_tgt_pc_a3 :
                     (CPU_valid_jump_a3 && CPU_is_jalr_a3) ? CPU_jalr_tgt_pc_a3 : CPU_inc_pc_a1;
         
         assign CPU_imem_rd_en_a0 = !CPU_reset_a0;
         assign CPU_imem_rd_addr_a0[31:0] = CPU_pc_a0[4+1:2];
         
      //_@1         
         assign CPU_instr_a1[31:0] = CPU_imem_rd_data_a1[31:0];
         assign CPU_inc_pc_a1[31:0] = CPU_pc_a1 + 32'd4;          
      // Decode   
         assign CPU_is_i_instr_a1 = CPU_instr_a1[6:2] == 5'b00000 ||
                       CPU_instr_a1[6:2] == 5'b00001 ||
                       CPU_instr_a1[6:2] == 5'b00100 ||
                       CPU_instr_a1[6:2] == 5'b00110 ||
                       CPU_instr_a1[6:2] == 5'b11001;
         assign CPU_is_r_instr_a1 = CPU_instr_a1[6:2] == 5'b01011 ||
                       CPU_instr_a1[6:2] == 5'b10100 ||
                       CPU_instr_a1[6:2] == 5'b01100 ||
                       CPU_instr_a1[6:2] == 5'b01101;                       
         assign CPU_is_b_instr_a1 = CPU_instr_a1[6:2] == 5'b11000;
         assign CPU_is_u_instr_a1 = CPU_instr_a1[6:2] == 5'b00101 ||
                       CPU_instr_a1[6:2] == 5'b01101;
         assign CPU_is_s_instr_a1 = CPU_instr_a1[6:2] == 5'b01000 ||
                       CPU_instr_a1[6:2] == 5'b01001;
         assign CPU_is_j_instr_a1 = CPU_instr_a1[6:2] == 5'b11011;
         
         assign CPU_imm_a1[31:0] = CPU_is_i_instr_a1 ? { {21{CPU_instr_a1[31]}} , CPU_instr_a1[30:20] } :
                      CPU_is_s_instr_a1 ? { {21{CPU_instr_a1[31]}} , CPU_instr_a1[30:25] , CPU_instr_a1[11:8] , CPU_instr_a1[7] } :
                      CPU_is_b_instr_a1 ? { {20{CPU_instr_a1[31]}} , CPU_instr_a1[7] , CPU_instr_a1[30:25] , CPU_instr_a1[11:8] , 1'b0} :
                      CPU_is_u_instr_a1 ? { CPU_instr_a1[31:12] , 12'b0} : 
                      CPU_is_j_instr_a1 ? { {12{CPU_instr_a1[31]}} , CPU_instr_a1[19:12] , CPU_instr_a1[20] , CPU_instr_a1[30:21] , 1'b0} : 32'b0;
         
         assign CPU_rs2_valid_a1 = CPU_is_r_instr_a1 || CPU_is_s_instr_a1 || CPU_is_b_instr_a1;
         assign CPU_rs1_valid_a1 = CPU_is_r_instr_a1 || CPU_is_s_instr_a1 || CPU_is_b_instr_a1 || CPU_is_i_instr_a1;
         assign CPU_rd_valid_a1 = CPU_is_r_instr_a1 || CPU_is_i_instr_a1 || CPU_is_u_instr_a1 || CPU_is_j_instr_a1;
         assign CPU_funct3_valid_a1 = CPU_is_r_instr_a1 || CPU_is_s_instr_a1 || CPU_is_b_instr_a1 || CPU_is_i_instr_a1;
         assign CPU_funct7_valid_a1 = CPU_is_r_instr_a1;
         
         //_?$rs2_valid
            assign CPU_rs2_a1[4:0] = CPU_instr_a1[24:20];
         //_?$rs1_valid
            assign CPU_rs1_a1[4:0] = CPU_instr_a1[19:15];
         //_?$rd_valid
            assign CPU_rd_a1[4:0] = CPU_instr_a1[11:7];
         //_?$funct3_valid
            assign CPU_funct3_a1[2:0] = CPU_instr_a1[14:12];
         //_?$funct7_valid
            assign CPU_funct7_a1[6:0] = CPU_instr_a1[31:25];
            
         assign CPU_opcode_a1[6:0] = CPU_instr_a1[6:0];
         
         assign CPU_dec_bits_a1[10:0] = {CPU_funct7_a1[5],CPU_funct3_a1,CPU_opcode_a1};
         
         // Branch Instruction
         assign CPU_is_beq_a1 = CPU_dec_bits_a1[9:0] == 10'b000_1100011;
         assign CPU_is_bne_a1 = CPU_dec_bits_a1[9:0] == 10'b001_1100011;
         assign CPU_is_blt_a1 = CPU_dec_bits_a1[9:0] == 10'b100_1100011;
         assign CPU_is_bge_a1 = CPU_dec_bits_a1[9:0] == 10'b101_1100011;
         assign CPU_is_bltu_a1 = CPU_dec_bits_a1[9:0] == 10'b110_1100011;
         assign CPU_is_bgeu_a1 = CPU_dec_bits_a1[9:0] == 10'b111_1100011;
         
         // Arithmetic Instruction
         assign CPU_is_add_a1 = CPU_dec_bits_a1 == 11'b0_000_0110011;
         assign CPU_is_addi_a1 = CPU_dec_bits_a1[9:0] == 10'b000_0010011;
         assign CPU_is_or_a1 = CPU_dec_bits_a1 == 11'b0_110_0110011;
         assign CPU_is_ori_a1 = CPU_dec_bits_a1[9:0] == 10'b110_0010011;
         assign CPU_is_xor_a1 = CPU_dec_bits_a1 == 11'b0_100_0110011;
         assign CPU_is_xori_a1 = CPU_dec_bits_a1[9:0] == 10'b100_0010011;
         assign CPU_is_and_a1 = CPU_dec_bits_a1 == 11'b0_111_0110011;
         assign CPU_is_andi_a1 = CPU_dec_bits_a1[9:0] == 10'b111_0010011;
         assign CPU_is_sub_a1 = CPU_dec_bits_a1 == 11'b1_000_0110011;
         assign CPU_is_slti_a1 = CPU_dec_bits_a1[9:0] == 10'b010_0010011;
         assign CPU_is_sltiu_a1 = CPU_dec_bits_a1[9:0] == 10'b011_0010011;
         assign CPU_is_slli_a1 = CPU_dec_bits_a1 == 11'b0_001_0010011;
         assign CPU_is_srli_a1 = CPU_dec_bits_a1 == 11'b0_101_0010011;
         assign CPU_is_srai_a1 = CPU_dec_bits_a1 == 11'b1_101_0010011;
         assign CPU_is_sll_a1 = CPU_dec_bits_a1 == 11'b0_001_0110011;
         assign CPU_is_slt_a1 = CPU_dec_bits_a1 == 11'b0_010_0110011;
         assign CPU_is_sltu_a1 = CPU_dec_bits_a1 == 11'b0_011_0110011;
         assign CPU_is_srl_a1 = CPU_dec_bits_a1 == 11'b0_101_0110011;
         assign CPU_is_sra_a1 = CPU_dec_bits_a1 == 11'b1_101_0110011;
         
         // Load Instruction
         assign CPU_is_load_a1 = CPU_dec_bits_a1[6:0] == 7'b0000011;
         
         // Store Instruction
         assign CPU_is_sb_a1 = CPU_dec_bits_a1[9:0] == 10'b000_0100011;
         assign CPU_is_sh_a1 = CPU_dec_bits_a1[9:0] == 10'b001_0100011;
         assign CPU_is_sw_a1 = CPU_dec_bits_a1[9:0] == 10'b010_0100011;
         
         // Jump Instruction
         assign CPU_is_lui_a1 = CPU_dec_bits_a1[6:0] == 7'b0110111;
         assign CPU_is_auipc_a1 = CPU_dec_bits_a1[6:0] == 7'b0010111;
         assign CPU_is_jal_a1 = CPU_dec_bits_a1[6:0] == 7'b1101111;
         assign CPU_is_jalr_a1 = CPU_dec_bits_a1[9:0] == 10'b000_1100111;
         
         assign CPU_is_jump_a1 = CPU_is_jal_a1 || CPU_is_jalr_a1;
         
      //_@2   
      // Register File Read
         assign CPU_rf_rd_en1_a2 = CPU_rs1_valid_a2;
         //_?$rf_rd_en1
            assign CPU_rf_rd_index1_a2[4:0] = CPU_rs1_a2[4:0];
         
         assign CPU_rf_rd_en2_a2 = CPU_rs2_valid_a2;
         //_?$rf_rd_en2
            assign CPU_rf_rd_index2_a2[4:0] = CPU_rs2_a2[4:0];
            
      // Branch Target PC       
         assign CPU_br_tgt_pc_a2[31:0] = CPU_pc_a2 + CPU_imm_a2;
      
      // Jump Target PC
         assign CPU_jalr_tgt_pc_a2[31:0] = CPU_src1_value_a2 + CPU_imm_a2;
         
      // Input signals to ALU
         assign CPU_src1_value_a2[31:0] = ((CPU_rd_a3 == CPU_rs1_a2) && CPU_rf_wr_en_a3) ? CPU_result_a3 : CPU_rf_rd_data1_a2[31:0];
         assign CPU_src2_value_a2[31:0] = ((CPU_rd_a3 == CPU_rs2_a2) && CPU_rf_wr_en_a3) ? CPU_result_a3 : CPU_rf_rd_data2_a2[31:0];
         
      //_@3   
         
      // ALU
         assign CPU_sltu_result_a3 = CPU_src1_value_a3 < CPU_src2_value_a3 ;
         assign CPU_sltiu_result_a3 = CPU_src1_value_a3 < CPU_imm_a3 ;
         
         assign CPU_result_a3[31:0] = CPU_is_addi_a3 ? CPU_src1_value_a3 + CPU_imm_a3 :
                         CPU_is_add_a3 ? CPU_src1_value_a3 + CPU_src2_value_a3 : 
                         CPU_is_or_a3 ? CPU_src1_value_a3 | CPU_src2_value_a3 : 
                         CPU_is_ori_a3 ? CPU_src1_value_a3 | CPU_imm_a3 :
                         CPU_is_xor_a3 ? CPU_src1_value_a3 ^ CPU_src2_value_a3 :
                         CPU_is_xori_a3 ? CPU_src1_value_a3 ^ CPU_imm_a3 :
                         CPU_is_and_a3 ? CPU_src1_value_a3 & CPU_src2_value_a3 :
                         CPU_is_andi_a3 ? CPU_src1_value_a3 & CPU_imm_a3 :
                         CPU_is_sub_a3 ? CPU_src1_value_a3 - CPU_src2_value_a3 :
                         CPU_is_slti_a3 ? ((CPU_src1_value_a3[31] == CPU_imm_a3[31]) ? CPU_sltiu_result_a3 : {31'b0,CPU_src1_value_a3[31]}) :
                         CPU_is_sltiu_a3 ? CPU_sltiu_result_a3 :
                         CPU_is_slli_a3 ? CPU_src1_value_a3 << CPU_imm_a3[5:0] :
                         CPU_is_srli_a3 ? CPU_src1_value_a3 >> CPU_imm_a3[5:0] :
                         CPU_is_srai_a3 ? ({{32{CPU_src1_value_a3[31]}}, CPU_src1_value_a3} >> CPU_imm_a3[4:0]) :
                         CPU_is_sll_a3 ? CPU_src1_value_a3 << CPU_src2_value_a3[4:0] :
                         CPU_is_slt_a3 ? ((CPU_src1_value_a3[31] == CPU_src2_value_a3[31]) ? CPU_sltu_result_a3 : {31'b0,CPU_src1_value_a3[31]}) :
                         CPU_is_sltu_a3 ? CPU_sltu_result_a3 :
                         CPU_is_srl_a3 ? CPU_src1_value_a3 >> CPU_src2_value_a3[5:0] :
                         CPU_is_sra_a3 ? ({{32{CPU_src1_value_a3[31]}}, CPU_src1_value_a3} >> CPU_src2_value_a3[4:0]) :
                         CPU_is_lui_a3 ? ({CPU_imm_a3[31:12], 12'b0}) :
                         CPU_is_auipc_a3 ? CPU_pc_a3 + CPU_imm_a3 :
                         CPU_is_jal_a3 ? CPU_pc_a3 + 4 :
                         CPU_is_jalr_a3 ? CPU_pc_a3 + 4 : 
                         (CPU_is_load_a3 || CPU_is_s_instr_a3) ? CPU_src1_value_a3 + CPU_imm_a3 : 32'bx;
                         
      // Register File Write
         assign CPU_rf_wr_en_a3 = (CPU_rd_valid_a3 && CPU_valid_a3 && CPU_rd_a3 != 5'b0) || CPU_valid_load_a5;
         //_?$rf_wr_en
            assign CPU_rf_wr_index_a3[4:0] = !CPU_valid_a3 ? CPU_rd_a5[4:0] : CPU_rd_a3[4:0];
      
         assign CPU_rf_wr_data_a3[31:0] = !CPU_valid_a3 ? CPU_ld_data_a5[31:0] : CPU_result_a3[31:0];
      
      // Branch
         assign CPU_taken_br_a3 = CPU_is_beq_a3 ? (CPU_src1_value_a3 == CPU_src2_value_a3) :
                     CPU_is_bne_a3 ? (CPU_src1_value_a3 != CPU_src2_value_a3) :
                     CPU_is_blt_a3 ? ((CPU_src1_value_a3 < CPU_src2_value_a3) ^ (CPU_src1_value_a3[31] != CPU_src2_value_a3[31])) :
                     CPU_is_bge_a3 ? ((CPU_src1_value_a3 >= CPU_src2_value_a3) ^ (CPU_src1_value_a3[31] != CPU_src2_value_a3[31])) :
                     CPU_is_bltu_a3 ? (CPU_src1_value_a3 < CPU_src2_value_a3) :
                     CPU_is_bgeu_a3 ? (CPU_src1_value_a3 >= CPU_src2_value_a3) : 1'b0;
                     
         assign CPU_valid_taken_br_a3 = CPU_valid_a3 && CPU_taken_br_a3;
         
      // Load
         assign CPU_valid_load_a3 = CPU_valid_a3 && CPU_is_load_a3;
         assign CPU_valid_a3 = !(CPU_valid_taken_br_a4 || CPU_valid_taken_br_a5 || CPU_valid_load_a4 || CPU_valid_load_a5 || CPU_valid_jump_a4 || CPU_valid_jump_a5);
      
      // Jump
         assign CPU_valid_jump_a3 = CPU_valid_a3 && CPU_is_jump_a3;
                  
      //_@4
         assign CPU_dmem_rd_en_a4 = CPU_valid_load_a4;
         assign CPU_dmem_wr_en_a4 = CPU_valid_a4 && CPU_is_s_instr_a4;
         assign CPU_dmem_addr_a4[3:0] = CPU_result_a4[5:2];
         assign CPU_dmem_wr_data_a4[31:0] = CPU_src2_value_a4[31:0];
         
      //_@5   
         assign CPU_ld_data_a5[31:0] = CPU_dmem_rd_data_a5[31:0];
         
      // Note: Because of the magic we are using for visualisation, if visualisation is enabled below,
      //       be sure to avoid having unassigned signals (which you might be using for random inputs)
      //       other than those specifically expected in the labs. You'll get strange errors for these.

         //`BOGUS_USE($is_beq $is_bne $is_blt $is_bge $is_bltu $is_bgeu)
         //`BOGUS_USE($is_sb $is_sh $is_sw)
   // Assert these to end simulation (before Makerchip cycle limit).
   /*SV_plus*/
      always @ (posedge clk) begin
         if (CPU_Xreg_value_a5[17] == (1+2+3+4+5+6+7+8+9))
            out = CPU_Xreg_value_a5[17];                
      end
   
   // Macro instantiations for:
   //  o instruction memory
   //  o register file
   //  o data memory
   //  o CPU visualization
   //_|cpu
      //_\source /raw.githubusercontent.com/shivanishah269/riscvcore/master/FPGAImplementation/riscvshelllib.tlv 16   // Instantiated from mythcore_test.tlv, 248 as: m4+imem(@1)
         // Instruction Memory containing program defined by m4_asm(...) instantiations.
         //_@1
            /*SV_plus*/
               // The program in an instruction memory.
               wire [31:0] instrs [0:10-1];
               assign instrs[0] = {7'b0000000, 5'd0, 5'd0, 3'b000, 5'd10, 7'b0110011}; 
               assign instrs[1] = {7'b0000000, 5'd0, 5'd10, 3'b000, 5'd14, 7'b0110011}; 
               assign instrs[2] = {12'b1010, 5'd10, 3'b000, 5'd12, 7'b0010011}; 
               assign instrs[3] = {7'b0000000, 5'd0, 5'd10, 3'b000, 5'd13, 7'b0110011}; 
               assign instrs[4] = {7'b0000000, 5'd14, 5'd13, 3'b000, 5'd14, 7'b0110011}; 
               assign instrs[5] = {12'b1, 5'd13, 3'b000, 5'd13, 7'b0010011}; 
               assign instrs[6] = {1'b1, 6'b111111, 5'd12, 5'd13, 3'b100, 4'b1100, 1'b1, 7'b1100011}; 
               assign instrs[7] = {7'b0000000, 5'd0, 5'd14, 3'b000, 5'd10, 7'b0110011}; 
               assign instrs[8] = {7'b0000000, 5'd10, 5'd0, 3'b010, 5'b10000, 7'b0100011}; 
               assign instrs[9] = {12'b10000, 5'd0, 3'b010, 5'd17, 7'b0000011}; 
                 
            for (imem = 0; imem <= 9; imem=imem+1) begin : L1_CPU_Imem //_/imem
               assign CPU_Imem_instr_a1[imem][31:0] = instrs[imem];
            end
            //_?$imem_rd_en
               assign CPU_imem_rd_data_a1[31:0] = CPU_imem_rd_addr_a1 < 10 ? CPU_Imem_instr_a1[CPU_imem_rd_addr_a1] : 32'b0;
          
      //_\end_source    // Args: (read stage)
      //_\source /raw.githubusercontent.com/shivanishah269/riscvcore/master/FPGAImplementation/riscvshelllib.tlv 31   // Instantiated from mythcore_test.tlv, 249 as: m4+rf(@2, @3)
         // Reg File
         //_@3
            for (xreg = 0; xreg <= 31; xreg=xreg+1) begin : L1_CPU_Xreg //_/xreg

               // For $wr.
               wire L1_wr_a3;

               assign L1_wr_a3 = CPU_rf_wr_en_a3 && (CPU_rf_wr_index_a3 != 5'b0) && (CPU_rf_wr_index_a3 == xreg);
               assign CPU_Xreg_value_a3[xreg][31:0] = CPU_reset_a3 ?   xreg           :
                              L1_wr_a3        ?   CPU_rf_wr_data_a3 :
                                             CPU_Xreg_value_a4[xreg][31:0];
            end
         //_@2
            //_?$rf_rd_en1
               assign CPU_rf_rd_data1_a2[31:0] = CPU_Xreg_value_a4[CPU_rf_rd_index1_a2];
            //_?$rf_rd_en2
               assign CPU_rf_rd_data2_a2[31:0] = CPU_Xreg_value_a4[CPU_rf_rd_index2_a2];
            //`BOGUS_USE(CPU_rf_rd_data1_a2 CPU_rf_rd_data2_a2) 
      //_\end_source  // Args: (read stage, write stage) - if equal, no register bypass is required
      //_\source /raw.githubusercontent.com/shivanishah269/riscvcore/master/FPGAImplementation/riscvshelllib.tlv 48   // Instantiated from mythcore_test.tlv, 250 as: m4+dmem(@4)
         // Data Memory
         //_@4
            for (dmem = 0; dmem <= 15; dmem=dmem+1) begin : L1_CPU_Dmem //_/dmem

               // For $wr.
               wire L1_wr_a4;

               assign L1_wr_a4 = CPU_dmem_wr_en_a4 && (CPU_dmem_addr_a4 == dmem);
               assign CPU_Dmem_value_a4[dmem][31:0] = CPU_reset_a4 ?   dmem :
                              L1_wr_a4        ?   CPU_dmem_wr_data_a4 :
                                             CPU_Dmem_value_a5[dmem][31:0];
                                        
            end
            //_?$dmem_rd_en
               assign CPU_dmem_rd_data_a4[31:0] = CPU_Dmem_value_a5[CPU_dmem_addr_a4];
            //`BOGUS_USE($dmem_rd_data)
      //_\end_source    // Args: (read/write stage)
endgenerate

     
//_\SV
   
   endmodule

```
* MYTH core testbench is given below:

```
`timescale 1ns / 1ps

//Verilog generated by VPR 8.0.0+unkown from post-place-and-route implementation
/*module core (
    input \clk ,
    input \reset ,
    output \out[0] ,
    output \out[1] ,
    output \out[2] ,
    output \out[3] ,
    output \out[4] ,
    output \out[5] ,
    output \out[6] ,
    output \out[7] 
);
*/

module test();

reg clk, reset;
wire [7:0] out;

//core uut (clk,reset,out[0],out[1],out[2],out[3],out[4],out[5],out[6],out[7]);
core uut (clk,reset,out);

//initial $sdf_annotate("/home/nanditha/Dropbox/vsd/fpga-vsd-slides/Day3/post_synth_files/up_counter_post_synthesis.sdf", uut);


initial
begin
    clk = 1'b0;
    reset = 1'b0;
    
    #10;
    reset = 1'b1;
    #50;
    reset = 1'b0;
end

always #5 clk = ~clk;
initial #200 $finish;

endmodule

```

![Screenshot from 2022-03-28 11-23-38](https://user-images.githubusercontent.com/92034288/160395909-53291bdd-d30e-452b-9ec4-23acf3848da9.png)
*Figure 12. Shows RVMYTH Core Simulation*

![Screenshot from 2022-03-28 12-09-02](https://user-images.githubusercontent.com/92034288/160396184-3478578e-8e5a-4a9b-9dac-6d8342107fdc.png)
*Figure 13. Shows RVMYTH Core Timing Analysis*


![Screenshot from 2022-03-28 16-51-11](https://user-images.githubusercontent.com/92034288/160396559-3e12e043-aee1-4cb3-9435-cc43c5745d5f.png)
*Figure 14. Shows RVMYTH Core Resource Utilization*

## Day 4: Introduction to SOFA FPGA Fabric IP

* Skywater Opensource FPGAs are a series of open-source FPGA IPs which uses the open-source Skywater 130nm PDK and OpenFPGA framework.
* To use SOFA Framework run the following commands:
    * git clone https://github.com/lnis-uofu/SOFA.git
    * cd FPGA1212_QLSOFA_HD_PNR
    * make runOpenFPGA
 * Add new designs in benchmark directory and update task_simulation.conf to run those designs


![fig15](https://user-images.githubusercontent.com/92034288/160433777-fb7439df-691a-4f09-9c77-bca96d712c03.png)

*Figure 15. Shows The Counter's Resource Utilization*

![fig16](https://user-images.githubusercontent.com/92034288/160433977-43838695-a3e4-408c-bd72-62b48308571f.png)

*Figure 16. Snapshot of The Generated Timing Reports*

![fig17](https://user-images.githubusercontent.com/92034288/160434029-ec32a6a2-d8c9-4f75-bad1-24bf3600d6f5.png)

*Figure 17. Snapshot of The generated post_synthesis File*

![Screenshot from 2022-03-28 00-33-57](https://user-images.githubusercontent.com/92034288/160405573-d33c14e1-daf2-4385-8168-0d0b15ffea33.png)

*Figure 18. Snapshot of The post_synthesis Counter Design on Vivado*

## Day 5: RISC-V Core on Custom SOFA Fabric

* Run rvmyth core on SOFA by adding core design in benchmarks and update the task_simulation.conf file accordingly.

![fig19](https://user-images.githubusercontent.com/92034288/160434141-ef3678a6-7db5-4427-897a-0c6562ac7731.png)

*Figure 19. Snapshot of The Rvmyth Core Successful Run*

![fig20](https://user-images.githubusercontent.com/92034288/160434169-07f5a618-d5b5-4c8c-860b-c1a2bf48dee4.png)

*Figure 20. Resource Utilization of the Rvmyth Core on SOFA*

![fig21](https://user-images.githubusercontent.com/92034288/160434201-539c306f-ef22-4076-8d3a-aca6395d113d.png)

*Figure 21. Snapshot of the Timing Reports of the Rvmyth Core on SOFA*

![fig22](https://user-images.githubusercontent.com/92034288/160434310-6b007a48-ae0f-4cd5-ae10-4032d5ffafbe.png)

*Figure 22. Snapshot of the Timing Reports of the Rvmyth Core on SOFA*

![fig23](https://user-images.githubusercontent.com/92034288/160434379-a23c48f0-5d8a-4e8c-a5d4-64fed4eaff7c.png)

*Figure 23. Snapshot of the Slack Information for Rvmyth Core on SOFA*


![Screenshot from 2022-03-28 00-27-50](https://user-images.githubusercontent.com/92034288/160414659-5f30ed26-1110-4ccd-beca-97f9a3ec01fd.png)

*Figure 24. Vivado Simulation for the Post-synthesis Netlist of Rvmyth Core*
