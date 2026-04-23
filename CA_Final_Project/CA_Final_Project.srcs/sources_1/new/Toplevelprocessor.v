`timescale 1ns / 1ps
// Course Project - TopLevelProcessor
// Integrates all datapath modules including 3 new instructions:
//   - XORI  (I-type, funct3=100)
//   - XOR   (R-type, funct3=100, funct7=0)
//   - BNE   (B-type, funct3=001)
// Program: Fibonacci series (Part C) + instruction extension tests (Part B)

module TopLevelProcessor(
    input clk,
    input reset
);
    // ---- Wires ----
    wire [63:0] PC_Out, PC_Plus4, BranchTarget, PC_Next;
    wire [31:0] Instruction;
    wire [6:0]  opcode, funct7;
    wire [4:0]  RS1, RS2, RD;
    wire [2:0]  funct3;
    wire [3:0]  Funct, Operation;
    wire [63:0] ImmData, ALU_B_In, ALU_Result, ReadData1, ReadData2, WriteData, ReadData;
    wire        Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
    wire [1:0]  ALUOp;
    wire        Zero, BranchSelect, PCSrc;
    // Data memory observation
    wire [63:0] w0,w1,w2,w3,w4,w5,w6,w7;

    // ---- IF ----
    pcAdder      PCA (.PC(PC_Out), .PC_Plus4(PC_Plus4));
    branchAdder  BRA (.PC(PC_Out), .Imm(ImmData), .BranchTarget(BranchTarget));
    assign PCSrc = Branch & BranchSelect;
    mux2         PC_MUX (.In0(PC_Plus4), .In1(BranchTarget), .PCSrc(PCSrc), .Out(PC_Next));
    ProgramCounter PC_REG (.clk(clk), .reset(reset), .PC_In(PC_Next), .PC_Out(PC_Out));
    Instruction_Memory IM (.Inst_Address(PC_Out), .Instruction(Instruction));

    // ---- ID ----
    instruction_parser IP (
        .instruction(Instruction), .opcode(opcode), .rd(RD),
        .funct3(funct3), .rs1(RS1), .rs2(RS2), .funct7(funct7)
    );
    immGen IG (.instruction(Instruction), .imm_data(ImmData));
    control_unit CU (
        .Opcode(opcode), .Branch(Branch), .MemRead(MemRead),
        .MemtoReg(MemtoReg), .ALUOp(ALUOp), .MemWrite(MemWrite),
        .ALUSrc(ALUSrc), .RegWrite(RegWrite)
    );
    registerFile RF (
        .WriteData(WriteData), .RS1(RS1), .RS2(RS2), .RD(RD),
        .RegWrite(RegWrite), .Clk(clk), .Reset(reset),
        .ReadData1(ReadData1), .ReadData2(ReadData2)
    );
    Branch_unit BU (.funct3(funct3), .ReadData1(ReadData1), .ReadData2(ReadData2), .branchSel(BranchSelect));

    // ---- EX ----
    mux ALU_SRC_MUX (.a(ReadData2), .b(ImmData), .sel(ALUSrc), .data_out(ALU_B_In));
    assign Funct = {Instruction[30], Instruction[14:12]};
    ALU_control ALUC (.ALUOp(ALUOp), .Funct(Funct), .Operation(Operation));
    ALU_64_bit  ALU  (.a(ReadData1), .b(ALU_B_In), .ALUOp(Operation), .Result(ALU_Result), .Zero(Zero));

    // ---- MEM ----
    Data_Memory DM (
        .Memory_Address(ALU_Result), .Write_Data(ReadData2),
        .clk(clk), .MemWrite(MemWrite), .MemRead(MemRead), .ReadData(ReadData),
        .w0(w0),.w1(w1),.w2(w2),.w3(w3),.w4(w4),.w5(w5),.w6(w6),.w7(w7)
    );

    // ---- WB ----
    mux WB_MUX (.a(ALU_Result), .b(ReadData), .sel(MemtoReg), .data_out(WriteData));

endmodule