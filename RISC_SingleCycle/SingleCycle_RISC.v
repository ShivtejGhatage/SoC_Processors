// Instruction Fetch

module ProgramCounter(branch, BranchInstr, address);
    input branch,clk;
    input [5:0] BranchInstr;
    input [5:0] PCplus4;
    output reg [5:0] address;


    initial begin
        address = 6'b0;
    end

    always @(posedge clk) begin
        if (branch) begin
            address <= BranchInstr;
        end else begin
            address <= PCplus4;
        end
    end

endmodule

module PC4adder(address, PCplus4);
    input [5:0] address;
    output [5:0] PCplus4;

    assign PCplus4 = address + 6'b000100;

endmodule

module InstructionMemory (address, instruction);
    input [5:0] address;
    output [31:0] instruction;
    
    reg [31:0] RAM [0:63];

    initial begin
    $readmemh("instructions.dat", RAM);
    end

    assign instruction = RAM[address];


endmodule

// Instruction Decode

module RegisterFile (clk,rs,rt,rd,wdat,wen,read1,read2);
    input clk;
    input [4:0] rs,rt,rd;
    input [31:0] wdat;
    input wen;

    output reg [31:0] read1,read2;

    reg [31:0] Registers [31:0];


    always @ (posedge clk) begin
        if (wen && rd != 0) begin
            Registers[rd] <= wdat;
            case(rd)
                5'b10000: $display("content of $s0 = %h", wdat);
                5'b10001: $display("content of $s1 = %h", wdat);
                5'b10010: $display("content of $s2 = %h", wdat);
                5'b10011: $display("content of $s3 = %h", wdat);
                5'b10100: $display("content of $s4 = %h", wdat);
                5'b01000: $display("content of $t0 = %h", wdat);
                5'b01001: $display("content of $t1 = %h", wdat);
                //default: $display("no");
            endcase
        end
    end

    always @(*) begin
        read1 = Registers[rs];
        read2 = Registers[rt];
    end

endmodule

// Execution

module Controller(opcode,f7,f3,ALUsel, wen, WBSel, ALUSrc, branch, memwriteen);
    input [6:0] opcode;
    input [6:0] f7;
    input [2:0] f3;
    output reg [3:0] ALUsel;
    output reg wen,WBSel, ALUSrc, branch, memwriteen;


    // send for slt sel[3] = 1

    always @(opcode) begin
        branch = 0;
        ALUSrc = 1;

        if (opcode[4:0] == 5'b10011) begin
            ALUsel = {f7[5], f3};
            ALUSrc = opcode[5];
        end

        if (opcode == 7'b1100011) begin
            branch = 1;
            case(f3)
                3'b000: ALUsel = 4'b1000;
                3'b001: ALUsel = 4'b1000;
                3'b100: ALUsel = 4'b0010;
                3'b101: ALUsel = 4'b1001;
                3'b110: ALUsel = 4'b1010;
                3'b111: ALUsel = 4'b1111;
            endcase
        end


    end

endmodule


module ImmediateGenerator(instr,immediate);
    input [31:0] instr;
    output reg [31:0] immediate;

    always @(*) begin
        case (instr[6:0])
            7'b0110111: immediate = {instr[31:12], 12'b0};
            7'b1101111: immediate = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 16'b0};
            7'b1100011: immediate = {20'b0, instr[31], instr[7], instr[30:25], instr[11:8]};
            7'b1100111: immediate = {20'b0, instr[31:20]};
            7'b0000011: immediate = {20'b0, instr[31:20]};
            7'b0010011: immediate = {20'b0, instr[31:20]};
            // 7'b0100011: immediate = {20'b0, instr[31:25], instr[11:7]};
            7'b0010011: immediate = {27'b0, instr[24:20]};

            default: immediate = 32'b0;

        endcase
    end

endmodule

module ImmediatePC4adder(immediate,PCplus4,BranchAddr);
    input [31:0] immediate;
    input [5:0] PCplus4;
    output [5:0] BranchAddr;

    assign BranchAddr = PCplus4 + immediate[5:0];

endmodule


module ALU ( a, b, imm, ALUSrc, sel, out);
    input [31:0] a;
    input [31:0] b;
    input [31:0] imm;
    input ALUSrc;
    input [3:0] sel;
    output reg [31:0] out;
    reg [31:0] bornotb, sum;
    reg [31:0] operand2;


    always @(*) begin

        operand2 = ALUSrc ? imm : b;

        bornotb = sel[3] ? ~operand2 : operand2;
        sum = a + bornotb + sel[3];

    end

    always @ (a or b or imm or sel)
    begin
        case (sel[3:0])
            4'b0000: out = sum;
            4'b1000: out = sum;
            4'b0001: out = a << (operand2[4:0]);
            4'b0010: out = ($signed(a) < $signed(operand2) ) ? {31'b0,1'b1} : 32'b0;
            4'b0011: out = (a<operand2) ? 1 : 0;
            4'b0100: out = a^operand2;
            4'b0101: out = a >> (operand2[4:0]);
            4'b1101: out = $signed(a) >>> (operand2[4:0]);
            4'b0110: out = a | operand2;
            4'b0111: out = a & operand2;

            4'b1001: out = ($signed(a) > $signed(operand2)) ? 32'b1 : 32'b0;
            4'b1010: out = ((a) <= (operand2)) ? 32'b1 : 32'b0;
            4'b1111: out = ((a) > (operand2)) ? 32'b1 : 32'b0;




            default : out = 32'b0;

        endcase
    end
endmodule

// Memory Access

module DataMemory(clk, address, wdat, wen, readD);
    input clk;
    input [31:0] address;
    input [31:0] wdat;
    input wen;
    output [31:0] readD;

    reg [31:0] DRAM [63:0];

    assign readD = DRAM[address[7:2]];

    always @(posedge clk) begin
        if (wen) begin
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
    wire [5:0] BranchInstr,address;

    wire [5:0] PCplus4;

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


    ProgramCounter PC (
        .branch(branch),
        .BranchInstr(BranchInstr),
        .address(address)
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

    Controller Ctrl (
        .opcode(instruction[6:0]),
        .f7(instruction[31:25]),
        .f3(instruction[14:12]),
        .ALUsel(ALUsel)
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
        .out(ALUResult)
    );


    DataMemory DM (
        .clk(clk),
        .address(ALUResult),
        .wdat(read2),
        .wen(memwriteen),
        .readD(loadfromMem)
    );

    WriteBackMUX WB (
        .ALUResult(ALUResult),
        .LoadData(loadfromMem),
        .WBSel(WBSel),
        .WriteBack(wdat)
    );



endmodule
