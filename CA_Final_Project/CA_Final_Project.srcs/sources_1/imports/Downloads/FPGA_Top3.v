`timescale 1ns / 1ps
// =============================================================================
// FPGA Top-Level Wrapper - 64-bit Single-Cycle RISC-V Processor
// Target Board : Digilent Basys3 (XC7A35T)
//
// HOW IT WORKS:
//   1. Press BTNC (centre) at any time to RESET everything.
//   2. Press BTNR (right) once to RUN - the processor executes all
//      instructions automatically at full speed until it finishes
//      (PC reaches the last instruction at address 72).
//   3. After completion, the 7-segment display scrolls through all
//      8 Fibonacci results in decimal, one per second:
//         F(0)=0  F(1)=1  F(2)=1  F(3)=2  F(4)=3  F(5)=5  F(6)=8  F(7)=13
//      Each value is shown with a leading digit index (1-8) so you know
//      which term you are reading, e.g. "1. 0", "2. 1" ... "8.13"
//
// LED INDICATORS:
//   LED[0]     - RUNNING  (processor is executing)
//   LED[1]     - DONE     (Fibonacci complete, display scrolling)
//   LED[15:8]  - PC[9:2]  (instruction word address, for debugging)
// =============================================================================

module FPGA_Top (
    input        clk_100MHz,
    input        btnC,          // Centre  = RESET
    input        btnR,          // Right   = RUN (start program)

    output [15:0] led,
    output [6:0]  seg,
    output        dp,
    output [3:0]  an
);

    // =========================================================================
    // 1. 1 Hz tick - used for display scrolling and as processor clock
    // =========================================================================
    reg [26:0] div_cnt;
    reg        clk_1hz;

    always @(posedge clk_100MHz or posedge btnC) begin
        if (btnC) begin
            div_cnt <= 27'd0;
            clk_1hz <= 1'b0;
        end else if (div_cnt == 27'd49_999_999) begin
            div_cnt <= 27'd0;
            clk_1hz <= ~clk_1hz;
        end else begin
            div_cnt <= div_cnt + 27'd1;
        end
    end

    // Rising edge of 1 Hz clock ? 1-cycle pulse on 100 MHz domain
    reg clk_1hz_r;
    wire tick_1hz;
    always @(posedge clk_100MHz or posedge btnC) begin
        if (btnC) clk_1hz_r <= 1'b0;
        else      clk_1hz_r <= clk_1hz;
    end
    assign tick_1hz = clk_1hz & ~clk_1hz_r;

    // =========================================================================
    // 2. BTNR debounce - 10 ms window on 100 MHz clock
    // =========================================================================
    reg [19:0] db_cnt;
    reg        db_stable, db_prev;
    wire       run_pulse;           // one-shot: triggers the program run

    always @(posedge clk_100MHz or posedge btnC) begin
        if (btnC) begin
            db_cnt    <= 20'd0;
            db_stable <= 1'b0;
            db_prev   <= 1'b0;
        end else begin
            if (btnR != db_stable)
                db_cnt <= db_cnt + 20'd1;
            else
                db_cnt <= 20'd0;
            if (db_cnt == 20'd999_999) begin
                db_stable <= btnR;
                db_cnt    <= 20'd0;
            end
            db_prev <= db_stable;
        end
    end
    assign run_pulse = db_stable & ~db_prev;  // rising edge of debounced BTNR

    // =========================================================================
    // 3. FSM  - IDLE ? RUNNING ? DONE
    //    RUNNING: clock the processor at 100 MHz until PC == 72 (last instr)
    //    DONE   : freeze processor, scroll Fibonacci on display
    // =========================================================================
    localparam IDLE    = 2'd0;
    localparam RUNNING = 2'd1;
    localparam DONE    = 2'd2;

    reg [1:0] state;
    wire [63:0] PC_Out;

    // PC=72 is address of last instruction (NOP after BNE test)
    // We stop one cycle after PC reaches 72 so the last write completes.
    wire prog_done = (PC_Out >= 64'd72);

    always @(posedge clk_100MHz or posedge btnC) begin
        if (btnC)
            state <= IDLE;
        else begin
            case (state)
                IDLE:    if (run_pulse)  state <= RUNNING;
                RUNNING: if (prog_done)  state <= DONE;
                DONE:    state <= DONE;   // stay here until reset
                default: state <= IDLE;
            endcase
        end
    end

    wire proc_running = (state == RUNNING);
    wire proc_done    = (state == DONE);

    // Processor clock: only tick when in RUNNING state
    wire proc_clk = clk_100MHz & proc_running;

    // =========================================================================
    // 4. Processor
    // =========================================================================
    wire [63:0] w0, w1, w2, w3, w4, w5, w6, w7;

    TopLevelProcessor_obs uProc (
        .clk    (proc_clk),
        .reset  (btnC),
        .w0(w0), .w1(w1), .w2(w2), .w3(w3),
        .w4(w4), .w5(w5), .w6(w6), .w7(w7),
        .PC_Out (PC_Out)
    );

    // =========================================================================
    // 5. Scroll counter - advances through F(0)..F(7) at 1 Hz when DONE
    // =========================================================================
    reg [2:0] fib_idx;      // 0..7

    always @(posedge clk_100MHz or posedge btnC) begin
        if (btnC)
            fib_idx <= 3'd0;
        else if (proc_done && tick_1hz)
            fib_idx <= fib_idx + 3'd1;  // wraps 7?0 automatically
    end

    // =========================================================================
    // 6. Select current Fibonacci word
    // =========================================================================
    reg [63:0] fib_word;
    always @(*) begin
        case (fib_idx)
            3'd0: fib_word = w0;
            3'd1: fib_word = w1;
            3'd2: fib_word = w2;
            3'd3: fib_word = w3;
            3'd4: fib_word = w4;
            3'd5: fib_word = w5;
            3'd6: fib_word = w6;
            3'd7: fib_word = w7;
            default: fib_word = 64'd0;
        endcase
    end

    // =========================================================================
    // 7. Binary ? BCD (double-dabble, 14-bit)
    // =========================================================================
    wire [13:0] bin_val = fib_word[13:0];

    reg [3:0] bcd3, bcd2, bcd1, bcd0;
    reg [13:0] sr;
    reg [15:0] bcds;
    integer    k;

    always @(*) begin
        bcds = 16'd0;
        sr   = bin_val;
        for (k = 0; k < 14; k = k + 1) begin
            if (bcds[3:0]   >= 4'd5) bcds[3:0]   = bcds[3:0]   + 4'd3;
            if (bcds[7:4]   >= 4'd5) bcds[7:4]   = bcds[7:4]   + 4'd3;
            if (bcds[11:8]  >= 4'd5) bcds[11:8]  = bcds[11:8]  + 4'd3;
            if (bcds[15:12] >= 4'd5) bcds[15:12] = bcds[15:12] + 4'd3;
            bcds = {bcds[14:0], sr[13]};
            sr   = sr << 1;
        end
        bcd0 = bcds[3:0];
        bcd1 = bcds[7:4];
        bcd2 = bcds[11:8];
        bcd3 = bcds[15:12];
    end

    // Leading-zero blank (keep ones always on)
    wire blank3 = (bcd3 == 4'd0);
    wire blank2 =  blank3 & (bcd2 == 4'd0);
    wire blank1 =  blank2 & (bcd1 == 4'd0);

    // =========================================================================
    // 8. Display content selector
    //    IDLE    ? show "----" (all dashes)
    //    RUNNING ? show "run." (animated - 'run' + spinning dot via dp)
    //    DONE    ? show Fibonacci value in decimal with index on left
    //
    //    Display layout when DONE:
    //      digit3(leftmost) = term index (1..8)
    //      digit2           = dash '-'
    //      digit1           = tens of fib value  (blanked if 0)
    //      digit0           = ones of fib value
    //
    //    e.g. fib_idx=0 ? "1 - 0"   fib_idx=6 ? "7 - 8"   fib_idx=7 ? "8 -13"
    // =========================================================================

    // 7-seg encodings (active-high, inverted at output)
    //                         {g,f,e,d,c,b,a}
    localparam SEG_DASH  = 7'b1000000;   // '-'
    localparam SEG_OFF   = 7'b0000000;   // blank
    localparam SEG_r     = 7'b1010000;   // 'r' (lowercase)
    localparam SEG_u     = 7'b0111110;   // 'u'
    localparam SEG_n     = 7'b1010100;   // 'n'

    function [6:0] digit_seg;
        input [3:0] d;
        case (d)
            4'd0: digit_seg = 7'b0111111;
            4'd1: digit_seg = 7'b0000110;
            4'd2: digit_seg = 7'b1011011;
            4'd3: digit_seg = 7'b1001111;
            4'd4: digit_seg = 7'b1100110;
            4'd5: digit_seg = 7'b1101101;
            4'd6: digit_seg = 7'b1111101;
            4'd7: digit_seg = 7'b0000111;
            4'd8: digit_seg = 7'b1111111;
            4'd9: digit_seg = 7'b1101111;
            default: digit_seg = 7'b0000000;
        endcase
    endfunction

    // =========================================================================
    // 9. Refresh counter ~763 Hz digit scan
    // =========================================================================
    reg [16:0] refresh_cnt;
    always @(posedge clk_100MHz or posedge btnC) begin
        if (btnC) refresh_cnt <= 17'd0;
        else      refresh_cnt <= refresh_cnt + 17'd1;
    end
    wire [1:0] digit_sel = refresh_cnt[16:15];

    // =========================================================================
    // 10. Digit mux - output per state
    // =========================================================================
    reg [6:0] seg_r;
    reg       dp_r;

    always @(*) begin
        dp_r  = 1'b0;   // dp off by default (inverted at output)
        seg_r = SEG_OFF;

        if (state == IDLE) begin
            // Show "----"
            seg_r = SEG_DASH;

        end else if (state == RUNNING) begin
            // Show "run " - r=dig3, u=dig2, n=dig1, blank=dig0
            case (digit_sel)
                2'b11: seg_r = SEG_r;
                2'b10: seg_r = SEG_u;
                2'b01: seg_r = SEG_n;
                2'b00: seg_r = SEG_OFF;
            endcase

        end else begin
            // DONE - show  [index+1] [-] [tens] [ones]
            case (digit_sel)
                2'b11: seg_r = digit_seg(fib_idx + 4'd1);      // term number 1..8
                2'b10: seg_r = SEG_DASH;                        // separator '-'
                2'b01: seg_r = blank1 ? SEG_OFF : digit_seg(bcd1); // tens (blanked if 0)
                2'b00: seg_r = digit_seg(bcd0);                 // ones (always shown)
            endcase
        end
    end

    assign an  = ~(4'b0001 << digit_sel);  // active-low anodes
    assign seg = ~seg_r;                    // invert for common-anode
    assign dp  = ~dp_r;

    // =========================================================================
    // 11. LEDs
    // =========================================================================
    assign led[0]    = proc_running;
    assign led[1]    = proc_done;
    assign led[7:2]  = 6'b0;
    assign led[15:8] = PC_Out[9:2];    // instruction word address

endmodule


// =============================================================================
// Observable wrapper - exposes w0..w7 and PC_Out
// =============================================================================
module TopLevelProcessor_obs (
    input        clk,
    input        reset,
    output [63:0] w0, w1, w2, w3, w4, w5, w6, w7,
    output [63:0] PC_Out
);
    wire [63:0] PC_Plus4, BranchTarget, PC_Next;
    wire [31:0] Instruction;
    wire [6:0]  opcode, funct7;
    wire [4:0]  RS1, RS2, RD;
    wire [2:0]  funct3;
    wire [3:0]  Funct, Operation;
    wire [63:0] ImmData, ALU_B_In, ALU_Result, ReadData1, ReadData2, WriteData, ReadData;
    wire        Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
    wire [1:0]  ALUOp;
    wire        Zero, BranchSelect, PCSrc;

    pcAdder      PCA (.PC(PC_Out),       .PC_Plus4(PC_Plus4));
    branchAdder  BRA (.PC(PC_Out),       .Imm(ImmData), .BranchTarget(BranchTarget));
    assign PCSrc = Branch & BranchSelect;
    mux2         PC_MUX (.In0(PC_Plus4), .In1(BranchTarget), .PCSrc(PCSrc), .Out(PC_Next));
    ProgramCounter PC_REG (.clk(clk),   .reset(reset), .PC_In(PC_Next), .PC_Out(PC_Out));
    Instruction_Memory IM (.Inst_Address(PC_Out), .Instruction(Instruction));

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

    mux ALU_SRC_MUX (.a(ReadData2), .b(ImmData), .sel(ALUSrc), .data_out(ALU_B_In));
    assign Funct = {Instruction[30], Instruction[14:12]};
    ALU_control ALUC (.ALUOp(ALUOp), .Funct(Funct), .Operation(Operation));
    ALU_64_bit  ALU  (.a(ReadData1), .b(ALU_B_In), .ALUOp(Operation), .Result(ALU_Result), .Zero(Zero));

    Data_Memory DM (
        .Memory_Address(ALU_Result), .Write_Data(ReadData2),
        .clk(clk), .MemWrite(MemWrite), .MemRead(MemRead), .ReadData(ReadData),
        .w0(w0),.w1(w1),.w2(w2),.w3(w3),.w4(w4),.w5(w5),.w6(w6),.w7(w7)
    );

    mux WB_MUX (.a(ALU_Result), .b(ReadData), .sel(MemtoReg), .data_out(WriteData));

endmodule