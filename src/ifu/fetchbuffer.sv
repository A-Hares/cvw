///////////////////////////////////////////
// FetchBuffer.sv
//
// Written: 
// Created: 23 March 2024
// Modified: 
//
// Purpose: Implements Fetch Buffer to fetch two cache lines in order to prevent spills.
//          The module takes the PCnextF/PCF checks if the line with that address is in the fetch buffer
//          if yes, the IFU uses that line for word selection from line 
//          if no, the line is fetched from the cache  
//          somewhere during operation, the fetch buffer would fetch the next line from the cache and put it into the other register
//          We may read 2-byte or 4-byte sized instructions, from the fetch buffer 
// Documentation: 
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// https://github.com/openhwgroup/cvw
//
// Copyright (C) 2021-24 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////

module FetchBuffer import cvw::*; 
    #(  parameter cvw_t P,
        parameter PA_BITS, LINELEN, WORDLEN, MUXINTERVAL) 
        (
            input   logic                   clk,
            input   logic                   reset,
            input   logic   [LINELEN-1:0]   ReadDataLine,
            input   logic   [PA_BITS-1:0]   PAdr,  
            input   logic                   Stall,             // Stall the cache, preventing new accesses. In-flight access finished but does not return to READY
            input   logic                   FlushStage,        // Pipeline flush of second stage (prevent writes and bus operations)
            output  logic   [PA_BITS-1:0]   PAdr_out,  
            output  logic                   PAdr_mux,
            output  logic   [WORDLEN-1:0]   ReadDataWord    
        );
    
    
        // Line parameters
        localparam [31:0]              nop = 32'h00000013;                       // instruction for NOP
        localparam                     WORDSPERLINE = LINELEN/MUXINTERVAL;
        
        // Fetch buffer registers
        logic   [LINELEN-1:0]       Line1       ,   Line2   ;
        logic                       Line1_Valid ,   Line2_Valid;
        logic   [PA_BITS-1:0]       Line1_PAdr  ,   Line2_PAdr;
        logic                       Line1_en    ,   Line2_en;
    
        // Control Signals
        logic                       ActiveLine;                             // (0) Line1 is active, (1) Line2 is active.
        logic                       LastW, LastHW;
        logic   [4:0]               ActiveLineCount;
        logic                       NextLineFetched;
        logic   [1:0]               LineExists;
    
        // Intermediate data
        logic   [LINELEN-1:0]       SelectedLine;                           // The line currently being read from
        logic   [MUXINTERVAL-1:0]   ExtraHW;                                // Extra Half word at the beginning of the next line
        logic   [PA_BITS-1:0]       PAdr_out_reg;
    
        ////////////////////////////////////////////////////////////////////////////////////////////////
        // Word Selection and Spill Correction
        ////////////////////////////////////////////////////////////////////////////////////////////////
    
        logic [LINELEN+(WORDLEN-MUXINTERVAL)-1:0]   ReadDataLinePad;
        logic [WORDLEN-1:0]                         ReadDataLineSets [(LINELEN/MUXINTERVAL)-1:0];
    
        assign ActiveLineCount = ActiveLine ? Line2_PAdr[5:1] : Line1_PAdr[5:1];
        assign LastHW       = & ActiveLineCount;
        assign LastW        = & ActiveLineCount[4:1];
        // Decide which line last half-word (of Non Active Line) is forwarded to be padded with Selected Line (Active Line)
        assign ExtraHW      = (ActiveLine & LastHW) ? Line2[LINELEN-1:LINELEN-MUXINTERVAL] : Line1[LINELEN-1:LINELEN-MUXINTERVAL]; 
    
        // Selects the active line and pads it with the Extra Half word
        assign SelectedLine = ActiveLine ? Line2 : Line1;
        assign ReadDataLinePad = {ExtraHW, SelectedLine};
    
        genvar index;
        for (index = 0; index < WORDSPERLINE; index++) begin :readdatalinesetsmux
            assign ReadDataLineSets[index] = ReadDataLinePad[(index*MUXINTERVAL)+WORDLEN-1 : (index*MUXINTERVAL)];
        end
    
        assign ReadDataWord = ReadDataLineSets[ActiveLineCount];
    
        ////////////////////////////////////////////////////////////////////////////////////////////////
        // State and Data Registers
        ////////////////////////////////////////////////////////////////////////////////////////////////
    
        typedef enum logic [2:0]{STATE_LOAD, STATE_READ} statetype;
        statetype Current_State, NextState;
    
        always_ff @(posedge clk)
            if (reset | FlushStage)     Current_State <= STATE_LOAD;
            else                        Current_State <= NextState; 
        
        always_ff @(posedge clk) begin
            if (reset | FlushStage) begin 
                {Line1_Valid, Line1_PAdr, Line1} = nop;
                {Line2_Valid, Line2_PAdr, Line2} = nop;
            end
            else begin
                if      (~ActiveLine )  begin  
                    if (~LastHW & ~LastW) Line1_PAdr <= PAdr;      
                    if      (Line1_en)     {Line1_Valid, Line1_PAdr, Line1} <= {1'b1, PAdr, ReadDataLine};
                    else if (Line2_en)     {Line2_Valid, Line2_PAdr, Line2} <= {1'b1, PAdr_out, ReadDataLine};
                end
                else if (ActiveLine     & LineExists[1])  begin      
                    Line2_PAdr <= PAdr;  
                    if      (Line1_en)     {Line1_Valid, Line1_PAdr, Line1} <= {1'b1, PAdr_out, ReadDataLine};
                end              
            end
        end
    
        ////////////////////////////////////////////////////////////////////////////////////////////////
        // Next State Logic
        ////////////////////////////////////////////////////////////////////////////////////////////////
    
        always_comb begin
            NextState = STATE_LOAD;
            Line1_en = 0;
            Line2_en = 0;
            case (Current_State)                                                                                        
                STATE_LOAD: begin  
                                if (~Stall) begin NextState = STATE_READ; Line1_en = 1; end
                            end
                STATE_READ: begin
                                NextState = STATE_READ;
                                if (NextLineFetched) begin
                                    if(~ActiveLine) begin Line2_en = 1;     end
                                    else            begin Line1_en = 1;     end
                                end
                                if (LineExists == 0) NextState = STATE_LOAD;     
                            end
                default:    NextState = STATE_LOAD;
            endcase
        end
    
        // ActiveLine register
        always_ff @(posedge clk) begin
            if (reset | FlushStage) begin       ActiveLine <= 0; end
            else if         (LineExists[0])     ActiveLine <= 0;
            else if         (LineExists[1])     ActiveLine <= 1;
        end
    
        // NextLineFetched Register
        always_ff @(posedge clk) begin
            if (reset | FlushStage) begin NextLineFetched <= 0; end
            else begin 
                if (PAdr_mux)           NextLineFetched <= 1;
                if (LastHW | LastW)     NextLineFetched <= 0;
            end
        end
    
        ////////////////////////////////////////////////////////////////////////////////////////////////
        // Next-Line Fetch Logic
        ////////////////////////////////////////////////////////////////////////////////////////////////
    
        assign PAdr_out_reg = PAdr  + ((ActiveLineCount == 2**4) ? 2**5 : 0);
    
        // LineExist Logic
        always_comb begin
            LineExists = 0;
            if      (   (PAdr[55:6] == Line1_PAdr[55:6] )   && Line1_Valid)     LineExists[0] = 1;
            else if (   (PAdr[55:6] == Line2_PAdr[55:6] )   && Line2_Valid)     LineExists[1] = 1;
        end
    
        // PAdr out and mux register
        always_ff @(posedge clk) begin
            if (reset | FlushStage) begin PAdr_out <= 0; PAdr_mux <= 0; end
            else begin 
                if (ActiveLineCount == 2**4)    begin  PAdr_mux <= 1;   PAdr_out <= PAdr_out_reg; end   
                else if (Stall & PAdr_mux)      begin  PAdr_mux <= 1;  end              
                else if (LastHW | LastW)        begin  PAdr_mux <= 0;  end
    
            end
        end
    

endmodule

/*
    
    AT reset, every thing is zero: lines, Padr_lines, and valid bits
    Operation:
        At first cycle without stalls or flushes, We get one line from the cache and its PAdr to save inside Line 1
        We start reading from this line by checking periodically if the PAdr of this instruction is in the line
        if the adr is in the line, the mux network selects the word desired
        if the word is compressed by checking least 2 bits, they go to the decompress module and then to reg file and CU
        At the middle of the current line, We can start reading the next line, this is done by adding the PAdr of the active line to 2^6 or sth to get the next line PAdr, i.e. the next set
            This new PAdr goes to IFU, and into the cache where it is selected instead of the PAdr from the immu
            The output then goes to the decode stage at the next cycle
        At this point a new line is available to be saved inside the non-active line 
        We keep reading the active line untill we reach its end as we should know from the PAdr input
            if we reach the last 16 bits in a line, we automatically get the first 16 bits from the non active line
            if the instruction is compressed, a mask may be applied on the final output instruction to zero other bits
            if not compressed, we already got the right instruction 
            Also at the next cycle, the active line would change and be the other line 


*/