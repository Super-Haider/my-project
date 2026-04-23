`timescale 1ns / 1ps
// Lab 11 - Task 1 Testbench
// Verifies: PC+4 sequential update, branch target update, immediate generation
module tb_Task1;

    // ---------- Clock & Reset ----------
    reg clk, reset;
    initial clk = 0;
    always #5 clk = ~clk;   // 10 ns period

    // ---------- Wires ----------
    wire [63:0] PC_Out;       // from ProgramCounter
    wire [63:0] PC_Plus4;     // from pcAdder
    wire [63:0] BranchTarget; // from branchAdder
    wire [63:0] PC_Next;      // from mux2
    wire [63:0] ImmOut;       // from immGen

    // ---------- PCSrc control (manually toggled in test) ----------
    reg PCSrc;

    // ---------- Immediate generation test instruction ----------
    // I-type:  ADDI x5, x0, 16   -> imm = 16
    //   imm[11:0] = 12'h010, rs1=x0, funct3=000, rd=x5, opcode=0010011
    reg [31:0] test_instr;

    // ---------- DUT instantiations ----------
    ProgramCounter PC_inst (
        .clk    (clk),
        .reset  (reset),
        .PC_In  (PC_Next),
        .PC_Out (PC_Out)
    );

    pcAdder PCA (
        .PC       (PC_Out),
        .PC_Plus4 (PC_Plus4)
    );

    // Use a fixed signed immediate for branch test: imm = 8
    // branchTarget = PC + (8 << 1) = PC + 16
    branchAdder BRA (
        .PC           (PC_Out),
        .Imm          (64'd8),      // test immediate
        .BranchTarget (BranchTarget)
    );

    mux2 MUX (
        .In0    (PC_Plus4),
        .In1    (BranchTarget),
        .PCSrc  (PCSrc),
        .Out    (PC_Next)
    );

    immGen IG (
        .instruction (test_instr),
        .imm_data    (ImmOut)
    );

    // ---------- Stimulus ----------
    integer i;
    initial begin
        $dumpfile("tb_Task1.vcd");
        $dumpvars(0, tb_Task1);

        // ---- Reset ----
        reset = 1; PCSrc = 0;
        test_instr = 32'h01000293; // ADDI x5, x0, 16  (imm=16)
        @(posedge clk); #1;
        reset = 0;

        // ---- Test: PC increments by 4 (PCSrc=0) ----
        $display("\n--- Sequential PC (PCSrc=0) ---");
        for (i = 0; i < 5; i = i + 1) begin
            @(posedge clk); #1;
            $display("  PC_Out=%0d  PC_Plus4=%0d", PC_Out, PC_Plus4);
        end

        // ---- Test: branch taken (PCSrc=1) ----
        $display("\n--- Branch taken (PCSrc=1) ---");
        PCSrc = 1;
        @(posedge clk); #1;
        $display("  PC_Out=%0d  BranchTarget=%0d  PC_Next=%0d",
                  PC_Out, BranchTarget, PC_Next);
        @(posedge clk); #1;
        $display("  PC_Out=%0d (should be previous BranchTarget)", PC_Out);

        // ---- Return to sequential ----
        PCSrc = 0;
        @(posedge clk); #1;
        $display("  PC_Out=%0d  PC_Plus4=%0d  (sequential again)", PC_Out, PC_Plus4);

        // ---- Test: Immediate generation ----
        $display("\n--- Immediate Generation ---");

        // I-type: imm = 16
        test_instr = 32'h01000293; // ADDI x5, x0, 16
        #1; $display("  I-type  instr=0x%08h  imm_data=%0d  (expected 16)",   test_instr, ImmOut);

        // S-type: imm = 4  (SW rs2, 4(rs1))
        // imm[11:5]=0000000, imm[4:0]=00100 -> instr[31:25]=7'b0, instr[11:7]=5'b00100
        test_instr = 32'h00202223; // SW x2, 4(x0)
        #1; $display("  S-type  instr=0x%08h  imm_data=%0d  (expected 4)",    test_instr, ImmOut);

        // B-type: imm = 8  (BEQ x0, x0, +16 bytes -> imm field =8)
        // BEQ x0,x0,8:  imm[12|10:5]=0000000, rs2=x0, rs1=x0, funct3=000, imm[4:1|11]=1000 0, opcode=1100011
        test_instr = 32'h00000463; // BEQ x0, x0, +8
        #1; $display("  B-type  instr=0x%08h  imm_data=%0d  (expected 4)",    test_instr, ImmOut);

        // Negative I-type: imm = -1
        test_instr = 32'hfff00293; // ADDI x5, x0, -1
        #1; $display("  I-type  instr=0x%08h  imm_data=%0d  (expected -1 / 0xFFF...FFFF)", test_instr, $signed(ImmOut));

        $display("\n--- All tests complete ---");
        #20 $finish;
    end

endmodule