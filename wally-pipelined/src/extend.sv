///////////////////////////////////////////
// extend.sv
//
// Written: David_Harris@hmc.edu 9 January 2021
// Modified: 
//
// Purpose: 
// 
// A component of the Wally configurable RISC-V project.
// 
// Copyright (C) 2021 Harvey Mudd College & Oklahoma State University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, 
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
// is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS 
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT 
// OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
///////////////////////////////////////////

`include "wally-macros.sv"

module extend #(parameter XLEN=32) (
  input  logic [31:7]     InstrDecompD,
  input  logic [2:0]      ImmSrcD,
  output logic [XLEN-1:0] ExtImmD);
 
  always_comb
    case(ImmSrcD) 
               // I-type 
      3'b000:   ExtImmD = {{(XLEN-12){InstrDecompD[31]}}, InstrDecompD[31:20]};  
               // S-type (stores)
      3'b001:   ExtImmD = {{(XLEN-12){InstrDecompD[31]}}, InstrDecompD[31:25], InstrDecompD[11:7]}; 
               // B-type (branches)
      3'b010:   ExtImmD = {{(XLEN-12){InstrDecompD[31]}}, InstrDecompD[7], InstrDecompD[30:25], InstrDecompD[11:8], 1'b0}; 
               // J-type (jal)
      3'b011:   ExtImmD = {{(XLEN-20){InstrDecompD[31]}}, InstrDecompD[19:12], InstrDecompD[20], InstrDecompD[30:21], 1'b0}; 
               // U-type (lui, auipc)
      3'b100:  ExtImmD = {{(XLEN-31){InstrDecompD[31]}}, InstrDecompD[30:12], 12'b0}; 
      /* verilator lint_off WIDTH */
      default: ExtImmD = 'bx; // undefined
      /* verilator lint_on WIDTH */
    endcase             
endmodule
