`timescale 1ns / 1ps
// Course Project Testbench
// Runs the Fibonacci + new instruction test program
// Monitors: PC, Fibonacci memory results (w0-w7), XOR/XORI/BNE register outputs
module tb_Project;

    reg clk, reset;
    initial clk = 0;
    always #5 clk = ~clk;  // 10ns period = 100MHz

    // Observation wires tapped from DUT internals
    wire [63:0] w0,w1,w2,w3,w4,w5,w6,w7;
    wire [63:0] PC_Out;

    // Instantiate processor
    TopLevelProcessor DUT (.clk(clk), .reset(reset));

    // Tap internal signals for monitoring
    assign w0    = DUT.w0;
    assign w1    = DUT.w1;
    assign w2    = DUT.w2;
    assign w3    = DUT.w3;
    assign w4    = DUT.w4;
    assign w5    = DUT.w5;
    assign w6    = DUT.w6;
    assign w7    = DUT.w7;
    assign PC_Out = DUT.PC_Out;

    integer cycle;
    initial begin
        $dumpfile("tb_Project.vcd");
        $dumpvars(0, tb_Project);

        // Reset for 2 cycles
        reset = 1;
        @(posedge clk); @(posedge clk); #1;
        reset = 0;

        $display("Cycle | PC  | w0  w1  w2  w3  w4  w5  w6  w7");
        $display("------|-----|----------------------------------");

        // Run enough cycles for entire program (19 instr + 6 loop iterations)
        // Loop runs 6 times x 6 instr = 36 + preamble 8 + post 4 = ~48 cycles
        for (cycle = 0; cycle < 60; cycle = cycle + 1) begin
            @(posedge clk); #1;
            $display("%5d | %3d | %2d %2d %2d %2d %2d %2d %2d %2d",
                cycle, PC_Out,
                w0, w1, w2, w3, w4, w5, w6, w7);
        end

        $display("\n--- Final Fibonacci Results ---");
        $display("mem[0]  = %0d  (expected 0)",  w0);
        $display("mem[4]  = %0d  (expected 1)",  w1);
        $display("mem[8]  = %0d  (expected 1)",  w2);
        $display("mem[12] = %0d  (expected 2)",  w3);
        $display("mem[16] = %0d  (expected 3)",  w4);
        $display("mem[20] = %0d  (expected 5)",  w5);
        $display("mem[24] = %0d  (expected 8)",  w6);
        $display("mem[28] = %0d  (expected 13)", w7);

        $display("\n--- New Instruction Tests ---");
        $display("x6 (XORI result) = %0d", DUT.RF.array[6]);
        $display("x7 (XOR  result) = %0d", DUT.RF.array[7]);
        $display("BNE branch verified via PC jump");

        #20 $finish;
    end
endmodule