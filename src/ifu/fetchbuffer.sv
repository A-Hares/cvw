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
    input   logic   [P.XLEN-1:0]    PCD,
    input   logic   [P.XLEN-1:0]    PCNextF,  
    input   logic                   Stall,             // Stall the cache, preventing new accesses. In-flight access finished but does not return to READY
    input   logic                   CacheStall,
    input   logic                   FlushStage,        // Pipeline flush of second stage (prevent writes and bus operations)
    output  logic                   StallFB,
    output  logic   [PA_BITS-1:0]   PAdr_out,  
    output  logic                   PAdr_mux,
    output  logic   [WORDLEN-1:0]   ReadDataWord,ReadDataWordNext    
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
    logic                       MidW, LastW, LastHW;
    logic   [4:0]               ActiveLineCount;
    logic   [1:0]               LineExists;

    // Intermediate data
    logic   [LINELEN-1:0]       SelectedLine;                           // The line currently being read from
    logic   [MUXINTERVAL-1:0]   ExtraHW;                                // Extra Half word at the beginning of the next line
    logic   [PA_BITS-1:0]       PAdr_out_reg;
    logic Stall2;
    assign LastHW       = & ActiveLineCount;
    assign LastW        = & ActiveLineCount[4:1];
    assign MidW         = (ActiveLineCount == 2**4);

    assign StallFB      =  CacheStall; //& (LineExists == 0);
    
    always_ff @(posedge clk) begin
        if (reset | FlushStage) Stall2 <= 0;
        else 
            if (LineExists == 0) Stall2 <= 1;
    end

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Word Selection and Spill Correction
    ////////////////////////////////////////////////////////////////////////////////////////////////

    logic [LINELEN+(WORDLEN-MUXINTERVAL)-1:0]   ReadDataLinePad;
    logic [WORDLEN-1:0]                         ReadDataLineSets [(LINELEN/MUXINTERVAL)-1:0];

    assign ActiveLineCount = ActiveLine ? Line2_PAdr[5:1] : Line1_PAdr[5:1];

    // Decide which line last half-word (of Non Active Line) is forwarded to be padded with Selected Line (Active Line)
    assign ExtraHW      = (ActiveLine) ? Line1[MUXINTERVAL-1:0] : Line2[MUXINTERVAL-1:0]; 

    // Selects the active line and pads it with the Extra Half word
    assign SelectedLine = ActiveLine ? Line2 : Line1;
    assign ReadDataLinePad = {ExtraHW, SelectedLine};

    genvar index;
    for (index = 0; index < WORDSPERLINE; index++) begin :readdatalinesetsmux
        assign ReadDataLineSets[index] = ReadDataLinePad[(index*MUXINTERVAL)+WORDLEN-1 : (index*MUXINTERVAL)];
    end

    assign ReadDataWord = ReadDataLineSets[ActiveLineCount];
    assign ReadDataWordNext = ReadDataLineSets[ActiveLineCount+2];

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // State and Data Registers
    ////////////////////////////////////////////////////////////////////////////////////////////////

    typedef enum logic [2:0]{STATE_INVALID, STATE_READ, STATE_PREFETCH} statetype;
    statetype Current_State, NextState;

    always_ff @(posedge clk)
        if (reset | FlushStage)     Current_State <= STATE_INVALID;
        else                        Current_State <= NextState; 
    
    always_ff @(posedge clk) begin
        if (reset | FlushStage) begin 
            {Line1_Valid, Line1_PAdr, Line1} = nop;
            {Line2_Valid, Line2_PAdr, Line2} = nop;
        end
        else begin    
            if      (Line1_en)     {Line1_Valid, Line1} <= {1'b1,  ReadDataLine};
            if      (Line2_en)     {Line2_Valid, Line2} <= {1'b1,  ReadDataLine};  

            if (~ (StallFB | Stall)) begin
                if (ActiveLine & MidW)
                    Line1_PAdr <= PAdr_out_reg;
                else if (LineExists[0] | (Line1_en & (Current_State == STATE_INVALID)))
                    Line1_PAdr <= PAdr;
                    
                if (~ActiveLine & MidW)
                    Line2_PAdr <= PAdr_out_reg;
                else if (LineExists[1] | (Line2_en & (Current_State == STATE_INVALID)))
                    Line2_PAdr <= PAdr;
            end
        end
    end

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Next State Logic
    ////////////////////////////////////////////////////////////////////////////////////////////////

    always_comb begin
        NextState = STATE_INVALID;
        Line1_en = 0;
        Line2_en = 0;
        PAdr_mux = 0;
        case (Current_State)                                                                                        
            STATE_INVALID:  begin  
                                if (~(StallFB | Stall) & ~FlushStage) begin 
                                    NextState = STATE_READ;
                                    Line1_en = 1; 
                                end
                            end
            STATE_READ:     begin
                                NextState = STATE_READ;
                                if (MidW)               NextState = STATE_PREFETCH;
                                if (LineExists == 0)    NextState = STATE_INVALID;     
                            end
            STATE_PREFETCH: begin
                                NextState = STATE_PREFETCH;
                                if (~(StallFB | Stall) & (ActiveLineCount >= 0))         NextState = STATE_READ;
                                
                                PAdr_mux = 1;
                                
                                if (ActiveLine)     Line1_en = 1; 
                                else                Line2_en = 1;

                                if (LineExists == 0)    NextState = STATE_INVALID;
                            end
            default:    NextState = STATE_INVALID;
        endcase
    end

    // ActiveLine register
    always_ff @(posedge clk) begin
        if (reset | FlushStage) begin       ActiveLine <= 0; end
        else if         (LineExists[0])     ActiveLine <= 0;
        else if         (LineExists[1])     ActiveLine <= 1;
    end

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Next-Line Fetch Logic
    ////////////////////////////////////////////////////////////////////////////////////////////////

    assign PAdr_out_reg = PAdr  + (MidW ? 2**4 : 0);

    // LineExist Logic
    always_comb begin
        LineExists = 0;
        if      (   (PAdr[PA_BITS-1:6] == Line1_PAdr[PA_BITS-1:6] )   && Line1_Valid)     LineExists[0] = 1;
        else if (   (PAdr[PA_BITS-1:6] == Line2_PAdr[PA_BITS-1:6] )   && Line2_Valid)     LineExists[1] = 1;
    end

    // PAdr out register
    always_ff @(posedge clk) begin
        if (reset | FlushStage) begin PAdr_out <= 0; end
        else begin 
            if (MidW) begin  
                PAdr_out <= PAdr_out_reg; 
            end   
        end
    end


endmodule

