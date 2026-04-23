`timescale 1ns / 1ps
// Lab 11 - Task 1
// 2-to-1 Mux: selects next PC value
//   PCSrc = 0 -> PC + 4 (sequential)
//   PCSrc = 1 -> Branch target
module mux2(
    input  [63:0] In0,      // PC + 4
    input  [63:0] In1,      // Branch target
    input         PCSrc,
    output [63:0] Out
);
    assign Out = PCSrc ? In1 : In0;
endmodule