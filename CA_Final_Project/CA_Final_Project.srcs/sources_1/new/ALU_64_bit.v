`timescale 1ns / 1ps
// Course Project - Updated ALU_64_bit
// New operations: XOR (4'b0100), SRL (4'b0101), SRA (4'b0111)
module ALU_64_bit(
    input  [63:0] a,
    input  [63:0] b,
    input  [3:0]  ALUOp,
    output reg [63:0] Result,
    output Zero
);
    always @(ALUOp, a, b) begin
        case (ALUOp)
            4'b0000: Result <= a & b;               // AND / ANDI
            4'b0001: Result <= a | b;               // OR  / ORI
            4'b0010: Result <= a + b;               // ADD / ADDI / LW / SW
            4'b0110: Result <= a - b;               // SUB / BEQ / BLT / BGE
            4'b1100: Result <= ~a & ~b;             // NOR
            4'b1000: Result <= a << b;              // SLL / SLLI
            4'b0100: Result <= a ^ b;               // XOR / XORI  <-- NEW
            4'b0101: Result <= a >> b;              // SRL / SRLI  <-- NEW
            4'b0111: Result <= $signed(a) >>> b[4:0]; // SRA       <-- NEW
            default: Result <= 64'b0;
        endcase
    end
    assign Zero = (Result == 0);
endmodule