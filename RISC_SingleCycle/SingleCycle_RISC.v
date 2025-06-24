// Instruction Fetch

module ProgramCounter(branch,zero,clk,PCplus4, BranchInstr, address);
    input branch,clk,zero;
    input [7:0] BranchInstr; // max 64 instructions
    input [7:0] PCplus4;
    output reg [7:0] address;


    initial begin
        address = 8'b0;
    end

    always @(posedge clk) begin
        if (branch & zero) begin
            address <= BranchInstr;
        end else begin
            address <= PCplus4;
        end
    end

endmodule

module PC4adder(address, PCplus4);
    input [7:0] address;
    output [7:0] PCplus4;

    assign PCplus4 = address + 8'b000100;

endmodule

module InstructionMemory (address, instruction);
    input [7:0] address;
    output reg [31:0] instruction;
    
    reg [31:0] RAM [0:8]; 

    initial begin
    $readmemh("instructions.hex", RAM);
    end

    always @(*) begin
        instruction = RAM[address[7:2]];  
        $display("Now performing instruction @ PC=%h: %h", address, instruction);
    end

endmodule

// Instruction Decode

module RegisterFile (clk,rs,rt,rd,wdat,wen,read1,read2);
    input clk;
    input [4:0] rs,rt,rd;
    input [31:0] wdat;
    input wen;

    output reg [31:0] read1,read2;

    reg [31:0] Registers [31:0];

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            Registers[i] = 32'b0;
    end

    always @ (posedge clk) begin
        if (wen && rd != 0) begin
            Registers[rd] <= wdat;          // Its always rd here not like in MIPS
            $display("WRITE: x%0d <= %h (wdat), wen=%b, from module at time %t", rd, wdat, wen, $time);

        end
    end

    always @(*) begin
        read1 = Registers[rs];
        read2 = Registers[rt];
    end

endmodule

// Execution

module ControlCenter(opcode,f7,f3,ALUsel, wen, WBSel, ALUSrc, branch, memwriteen);
    input [6:0] opcode;
    input [6:0] f7;
    input [2:0] f3;
    output reg [3:0] ALUsel;
    output reg wen,WBSel, ALUSrc, branch, memwriteen;


    // send for slt sel[3] = 1

    always @(*) begin
        branch = 0; //reset
        ALUSrc = 1;
        memwriteen = 0;
        WBSel = 0;
        wen = 0;

        //ALU stuff
        if (opcode[4:0] == 5'b10011) begin
            ALUsel = {f7[5], f3};
            ALUSrc = opcode[5];     //if 1 then b otherwise immediate
            wen = 1;
        end

        //Branch stuff
        else if (opcode == 7'b1100011) begin
            branch = 1;
            case(f3)
                3'b000: ALUsel = 4'b1000;
                3'b001: ALUsel = 4'b1110;
                3'b100: ALUsel = 4'b0010;
                3'b101: ALUsel = 4'b1001;
                3'b110: ALUsel = 4'b1010;
                3'b111: ALUsel = 4'b1111;
            endcase
        end

        //Store word
        else if (opcode == 7'b0100011) begin
            memwriteen = 1;
        end 

        // Load full word and half word
        else if (opcode == 7'b0000011) begin
            WBSel = 1;
            wen = 1;
            ALUSrc = 0;
            ALUsel = 4'b0000;
        end


    end

endmodule


module ImmediateGenerator(instr,immediate);
    input [31:0] instr;
    output reg [31:0] immediate;

    always @(*) begin
        case (instr[6:0])
            7'b0110111: immediate = {instr[31:12], 12'b0};
            7'b0010111: immediate = {instr[31:12], 12'b0};
            7'b1101111: immediate = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            7'b1100011: immediate = {19'b0, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            7'b1100111: immediate = {20'b0, instr[31:20]};
            7'b0000011: immediate = {20'b0, instr[31:20]};
            7'b0010011: immediate = {20'b0, instr[31:20]};
            7'b0100011: immediate = {20'b0, instr[31:25], instr[11:7]};
            7'b0010011: immediate = {27'b0, instr[24:20]};

            default: immediate = 32'b0;

        endcase
    end

endmodule

module ImmediatePC4adder(immediate,PCplus4,BranchAddr);
    input [31:0] immediate;
    input [7:0] PCplus4;
    output [7:0] BranchAddr;

    assign BranchAddr = PCplus4 + immediate[7:0];

endmodule


module ALU ( a, b, imm, ALUSrc, sel, out, zero);
    input [31:0] a;
    input [31:0] b;
    input [31:0] imm;
    input ALUSrc;
    input [3:0] sel;
    output reg [31:0] out;
    output reg zero;
    reg [31:0] bornotb, sum;
    reg [31:0] operand2;


    always @(*) begin
        operand2 = ALUSrc ? b : imm;

        bornotb = sel[3] ? ~operand2 : operand2;
        sum = a + bornotb + sel[3];         // Just for sub

    end

    always @ (a or b or imm or sel)
    begin

        case (sel[3:0])
            4'b0000: out = sum;
            4'b1000: begin
                out = sum;
                zero = (sum == 0);
            end
            4'b0001: out = a << (operand2[4:0]);
            4'b0010: begin
                out = ($signed(a) < $signed(operand2) ) ? 1 : 0; //not all bits like I thought
                zero = out[0];
            end
            4'b0011: out = (a<operand2) ? 1 : 0;
            4'b0100: out = a^operand2;
            4'b0101: out = a >> (operand2[4:0]);
            4'b1101: out = $signed(a) >>> (operand2[4:0]);
            4'b0110: out = a | operand2;
            4'b0111: out = a & operand2;

            // Just made new ones for branching
            4'b1110: zero = (a != operand2);
            4'b1001: zero = ($signed(a) > $signed(operand2));
            4'b1010: zero = ((a) <= (operand2));
            4'b1111: zero = ((a) > (operand2));




            default : out = 32'b0;

        endcase

        $display("ALU: a=%h, b(or immediate)=%h, sel=%b, result=%h", a, operand2, sel, out);

    end
endmodule

// Memory Access

module DataMemory(clk, address, wdat, memwriteen, readD);
    input clk;
    input [31:0] address;
    input [31:0] wdat;
    input memwriteen;
    output [31:0] readD;

    reg [31:0] DRAM [63:0];

    integer i;
    initial begin
        for (i = 0; i < 64; i = i + 1)
            DRAM[i] = 32'b0;
    end

    assign readD = DRAM[address[7:2]];

    always @(posedge clk) begin
        if (memwriteen) begin
            DRAM[address[7:2]] <= wdat;
            $display("Address: %h now has data: %h", address[31:2],wdat);
        end
    end
endmodule

// WriteBack

module WriteBackMUX(ALUResult, LoadData, WBSel, WriteBack);
    input [31:0] ALUResult, LoadData;
    input WBSel;

    output reg [31:0] WriteBack;

    always @(*) begin
        WriteBack = WBSel ? LoadData : ALUResult;
    end

endmodule



// RISC

module Processor (clk);
    input clk;

    wire branch;
    wire [7:0] BranchInstr,address;

    wire [7:0] PCplus4;

    wire [31:0] instruction;

    wire wen;
    wire [31:0] wdat,read1,read2;

    wire [3:0] ALUsel;
    wire ALUSrc;
    wire[31:0] immExtended;
    wire [31:0] ALUResult;

    wire memwriteen;

    wire [31:0] loadfromMem;

    wire WBSel;

    wire zero;


    ProgramCounter PC (
        .branch(branch),
        .BranchInstr(BranchInstr),
        .address(address),
        .zero(zero),
        .clk(clk),
        .PCplus4(PCplus4)
    );


    PC4adder pc4 (
        .address(address),
        .PCplus4(PCplus4)
    );



    InstructionMemory IM (
        .address(address),
        .instruction(instruction)
    );

    RegisterFile RF (
        .clk(clk),
        .rs(instruction[19:15]),
        .rt(instruction[24:20]),
        .rd(instruction[11:7]),
        .wdat(wdat),
        .wen(wen),
        .read1(read1),
        .read2(read2)
    );

    ControlCenter Ctrl (
        .opcode(instruction[6:0]),
        .f7(instruction[31:25]),
        .f3(instruction[14:12]),
        .ALUsel(ALUsel),
        .memwriteen(memwriteen),
        .wen(wen),
        .WBSel(WBSel), 
        .branch(branch), 
        .ALUSrc(ALUSrc)
    );

    ImmediateGenerator ImG (
        .instr(instruction),
        .immediate(immExtended)
    );

    ImmediatePC4adder IPC4(
        .immediate(immExtended),
        .PCplus4(PCplus4),
        .BranchAddr(BranchInstr)
    );


    ALU alu (
        .a(read1),
        .b(read2),
        .imm(immExtended),
        .ALUSrc(ALUSrc),
        .sel(ALUsel),
        .out(ALUResult),
        .zero(zero)

    );


    DataMemory DM (
        .clk(clk),
        .address(ALUResult),
        .wdat(read2),
        .memwriteen(memwriteen),
        .readD(loadfromMem)
    );

    WriteBackMUX WB (
        .ALUResult(ALUResult),
        .LoadData(loadfromMem),
        .WBSel(WBSel),
        .WriteBack(wdat)
    );



endmodule



module Testbench;

    // Clock signal
    reg clk;

    // Instantiate the processor
    Processor dut (
        .clk(clk)
    );

    // Clock generation: 10ns period
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // Toggle every 5ns → 10ns period
    end

    // Simulation control
    initial begin
        // Optional: dump variables
        $display("Simulation starting...");
        $dumpfile("dump.vcd");
        $dumpvars(0, dut);  // dumps all vars in Processor

        #100;  // Run for 1000ns (100 clock cycles)
        $display("Simulation done.");
        $finish;
    end

endmodule
