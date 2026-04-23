`timescale 1ns / 1ps
// Lab 11 - Task 1
// PC Adder: computes PC + 4 (next sequential instruction address)
module pcAdder(
    input  [63:0] PC,
    output [63:0] PC_Plus4
);
    assign PC_Plus4 = PC + 64'd4;
endmodule