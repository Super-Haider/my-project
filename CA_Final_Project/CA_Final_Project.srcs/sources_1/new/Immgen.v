`timescale 1ns / 1ps
// Lab 11 - Task 1
// Immediate Generator: supports I, S, and B-type RISC-V immediate formats
module immGen(
    input  [31:0] instruction,
    output reg [63:0] imm_data
);
    wire [1:0] code = instruction[6:5];

    always @(*) begin
        case (code)
            // I-type (opcode bits [6:5] = 00): loads, ADDI, etc.
            2'b00 : imm_data = {{52{instruction[31]}}, instruction[31:20]};

            // S-type (opcode bits [6:5] = 01): stores
            2'b01 : imm_data = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};

            // B-type (opcode bits [6:5] = 11): branches
            2'b11 : imm_data = {{52{instruction[31]}}, instruction[31], instruction[7],
                                  instruction[30:25], instruction[11:8]};

            default: imm_data = 64'b0;
        endcase
    end
endmodule