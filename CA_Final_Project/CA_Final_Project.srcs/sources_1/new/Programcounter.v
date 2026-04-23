`timescale 1ns / 1ps
// Lab 11 - Task 1
// Program Counter: stores and updates PC on every positive clock edge
module ProgramCounter(
    input        clk,
    input        reset,
    input  [63:0] PC_In,
    output reg [63:0] PC_Out
);
    always @(posedge clk) begin
        if (reset)
            PC_Out <= 64'b0;
        else
            PC_Out <= PC_In;
    end
endmodule