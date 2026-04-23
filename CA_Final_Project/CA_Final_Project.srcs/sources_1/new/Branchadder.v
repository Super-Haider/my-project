`timescale 1ns / 1ps
// Lab 11 - Task 1
// Branch Adder: computes PC + (sign-extended immediate << 1)
// Per RISC-V spec, B-type immediates encode multiples of 2;
// the immGen already gives the raw imm field, so we shift left by 1 here.
module branchAdder(
    input  [63:0] PC,
    input  [63:0] Imm,          // sign-extended immediate from immGen
    output [63:0] BranchTarget
);
    assign BranchTarget = PC + (Imm << 1);
endmodule