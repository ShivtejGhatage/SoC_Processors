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

module PC4adder(address, IF_PCplus4);
    input [7:0] address;
    output [7:0] IF_PCplus4;

    assign IF_PCplus4 = address + 8'b000100;

endmodule

module InstructionMemory (address, IF_instruction);
    input [7:0] address;
    output reg [31:0] IF_instruction;
    
    reg [31:0] RAM [0:8]; 

    initial begin
    $readmemh("instructions.hex", RAM);
    end

    always @(*) begin
        IF_instruction = RAM[address[7:2]];  
        $display("Now Fetched instruction @ PC=%h: %h", address, IF_instruction);
    end

endmodule

// IF/ID Register

module IFregisterID (clk,reset, IF_PCplus4, IF_instruction,ID_Instruction,ID_PCplus4);
    input [31:0] IF_instruction;
    input [7:0] IF_PCplus4;
    input clk;
    input reset;


    output reg [31:0] ID_Instruction;
    output reg [7:0] ID_PCplus4;


    always @(posedge(clk) or posedge(reset)) begin
        if (reset) begin
            ID_Instruction <= 32'b0;
            ID_PCplus4 <= 32'b0;
        end
        else begin
            ID_Instruction <= IF_instruction;
            ID_PCplus4 <= IF_PCplus4;
        end
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
        $display("READ: rs=x%0d -> %h, rt=x%0d -> %h", rs, read1, rt, read2);

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


module IDregisterEX (clk, ID_PCplus4, ID_Instruction,EX_ALUsel,EX_Instruction,reset,ID_Immediate, ID_BranchAddr, ALUsel, wen, WBSel, ALUSrc, branch, memwriteen,ID_read1,ID_read2,EX_ALUSrc,EX_wen,EX_WBSel,EX_branch,EX_memwriteen,EX_read1,EX_read2, EX_Immediate, EX_PCplus4, EX_BranchAddr);
    input [7:0] ID_PCplus4;
    input [31:0] ID_Immediate;

    input [7:0] ID_BranchAddr;
    output reg [7:0] EX_BranchAddr;

    input clk;
    input reset;
    input ALUSrc,wen,WBSel,branch,memwriteen;
    input [3:0] ALUsel;

    input [31:0] ID_read1,ID_read2;

    input [31:0] ID_Instruction;
    


    output reg [7:0] EX_PCplus4;
    output reg [31:0] EX_Immediate;

    output reg EX_ALUSrc,EX_wen,EX_WBSel,EX_branch,EX_memwriteen;
    output reg [3:0] EX_ALUsel;
    output reg [31:0] EX_Instruction;

    output reg [31:0] EX_read1,EX_read2;

    always @(posedge(clk) or posedge(reset)) begin
        if (reset) begin
            EX_Immediate <= 32'b0;
            EX_PCplus4 <= 32'b0;
            EX_ALUSrc <= 1'b0;
            EX_wen <= 1'b0;
            EX_WBSel <= 1'b0;
            EX_branch <= 1'b0;
            EX_memwriteen <= 1'b0;
            EX_ALUsel <= 4'b0;
            EX_BranchAddr <= 8'b0;
            EX_read1 <= 32'b0;
            EX_read2 <= 32'b0;
            EX_Instruction <= 32'b0;
        end
        else begin
            EX_Immediate <= ID_Immediate;
            EX_PCplus4 <= ID_PCplus4;
            EX_ALUSrc <= ALUSrc;
            EX_wen <= wen;
            EX_WBSel <= WBSel;
            EX_branch <= branch;
            EX_memwriteen <= memwriteen;
            EX_ALUsel <= ALUsel;
            EX_BranchAddr <= ID_BranchAddr;
            EX_read1 <= ID_read1;
            EX_read2 <= ID_read2;
            EX_Instruction <= ID_Instruction;
        end
    end

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
        zero = 0;
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



module EXregisterMEM (clk, EX_PCplus4,MEM_Instruction, EX_Instruction,reset, EX_wdat, EX_wen, EX_WBSel, EX_branch, EX_memwriteen, EX_ALUResult, MEM_wdat, MEM_wen, MEM_WBSel, MEM_branch, MEM_memwriteen, MEM_PCplus4, MEM_ALUResult);
    input [7:0] EX_PCplus4;
    input clk;
    input reset;
    input EX_wen,EX_WBSel,EX_branch,EX_memwriteen;
    input [31:0] EX_ALUResult;
    input [31:0] EX_wdat;

    input [31:0] EX_Instruction;
    output reg [31:0] MEM_Instruction;

    output reg [7:0] MEM_PCplus4;
    output reg MEM_wen,MEM_WBSel,MEM_branch,MEM_memwriteen;
    output reg [31:0] MEM_ALUResult;
    output reg [31:0] MEM_wdat;

    always @(posedge(clk) or posedge(reset)) begin
        if (reset) begin
            MEM_PCplus4 <= 32'b0;
            MEM_ALUResult <= 32'b0;
            MEM_wdat <= 32'b0;
            MEM_wen <= 1'b0;
            MEM_WBSel <= 1'b0;
            MEM_branch <= 1'b0;
            MEM_memwriteen <= 1'b0;
            MEM_Instruction <= 32'b0;
        end
        else begin
            MEM_PCplus4 <= EX_PCplus4;
            MEM_ALUResult <= EX_ALUResult;
            MEM_wdat <= EX_wdat;
            MEM_wen <= EX_wen;
            MEM_WBSel <= EX_WBSel;
            MEM_branch <= EX_branch;
            MEM_memwriteen <= EX_memwriteen;
            MEM_Instruction <= EX_Instruction;
        end
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



module MEMregisterWB (clk, reset,MEM_out,MEM_Instruction,WB_Instruction, MEM_ALUResult, MEM_wen, MEM_WBSel, WB_readD, WB_ALUResult, WB_wen, WB_WBSel);
    input clk;
    input reset;
    input MEM_wen,MEM_WBSel;
    input [31:0] MEM_ALUResult;
    input [31:0] MEM_out;
    

    input [31:0] MEM_Instruction;
    output reg [31:0] WB_Instruction;


    output reg WB_wen,WB_WBSel;
    output reg [31:0] WB_ALUResult;
    output reg [31:0] WB_readD;

    always @(posedge(clk) or posedge(reset)) begin
        if (reset) begin
            WB_wen <= 1'b0;
            WB_WBSel <= 1'b0;
            WB_ALUResult <= 32'b0;
            WB_readD <= 32'b0;
            WB_Instruction <= 32'b0;
        end
        else begin
            WB_wen <= MEM_wen;
            WB_WBSel <= MEM_WBSel;
            WB_ALUResult <= MEM_ALUResult;
            WB_readD <= MEM_out;
            WB_Instruction <= MEM_Instruction;
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
        //$display("WRITEBACK: wdat=%h to rd=x%0d, wen=%b", wdat, WB_Instruction[11:7], WB_wen);
    end

endmodule



// RISC

module Processor (clk,ID_Instruction,EX_Instruction,instruction,MEM_Instruction,WB_Instruction);
    input clk;
    

    wire branch;
    wire [7:0] BranchInstr,address;

    wire [7:0] PCplus4;

    output [31:0] instruction;

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


    // Registers
    output [31:0] ID_Instruction;
    wire [7:0] ID_PCplus4;

    wire [7:0] EX_PCplus4;
    wire [31:0] EX_Immediate;
    wire [7:0] EX_BranchAddr;
    output [31:0] EX_Instruction;
    wire [31:0] EX_read1;
    wire [31:0] EX_read2;
    wire [31:0] EX_ALUResult;
    wire [31:0] WB_readD;
    wire [31:0] WB_ALUResult;


    wire EX_ALUSrc,EX_wen,EX_WBSel,EX_branch,EX_memwriteen;
    wire [3:0] EX_ALUsel;

    wire [7:0] MEM_PCplus4;
    wire MEM_wen,MEM_WBSel,MEM_branch,MEM_memwriteen;
    wire [31:0] MEM_ALUResult;
    wire [31:0] MEM_wdat;
    output [31:0] WB_Instruction;
    output [31:0] MEM_Instruction;

    


    ProgramCounter PC (
        .branch(EX_branch),
        .BranchInstr(EX_BranchAddr),
        .address(address),
        .zero(zero),
        .clk(clk),
        .PCplus4(PCplus4)
    );


    PC4adder pc4 (
        .address(address),
        .IF_PCplus4(PCplus4)
    );



    InstructionMemory IM (
        .address(address),
        .IF_instruction(instruction)
    );

    IFregisterID IF_R_ID(
        .IF_instruction(instruction),
        .clk(clk),
        .reset(ID_reset),
        .IF_PCplus4(PCplus4),
        .ID_Instruction(ID_Instruction),
        .ID_PCplus4(ID_PCplus4)
    );

    RegisterFile RF (
        .clk(clk),
        .rs(ID_Instruction[19:15]),
        .rt(ID_Instruction[24:20]),
        .rd(WB_Instruction[11:7]),
        .wdat(wdat),
        .wen(WB_wen),
        .read1(read1),
        .read2(read2)
    );

    ControlCenter Ctrl (
        .opcode(ID_Instruction[6:0]),
        .f7(ID_Instruction[31:25]),
        .f3(ID_Instruction[14:12]),
        .ALUsel(ALUsel),
        .memwriteen(memwriteen),
        .wen(wen),
        .WBSel(WBSel), 
        .branch(branch), 
        .ALUSrc(ALUSrc)
    );

    ImmediateGenerator ImG (
        .instr(ID_Instruction),
        .immediate(immExtended)
    );

    ImmediatePC4adder IPC4(
        .immediate(immExtended),
        .PCplus4(ID_PCplus4),
        .BranchAddr(BranchInstr)
    );



    IDregisterEX ID_R_EX(
        .clk(clk),
        .reset(EX_reset),

        .ID_Instruction(ID_Instruction),
        .EX_Instruction(EX_Instruction),

        .ID_PCplus4(ID_PCplus4),
        .EX_PCplus4(EX_PCplus4),
        .ID_Immediate(immExtended),
        .EX_Immediate(EX_Immediate),
        .ID_BranchAddr(BranchInstr),
        .EX_BranchAddr(EX_BranchAddr),
        .ALUsel(ALUsel),
        .wen(wen),
        .WBSel(WBSel),
        .branch(branch), 
        .ALUSrc(ALUSrc),
        .memwriteen(memwriteen),
        .ID_read1(read1),
        .ID_read2(read2),

        .EX_wen(EX_wen),
        .EX_WBSel(EX_WBSel),
        .EX_branch(EX_branch), 
        .EX_ALUSrc(EX_ALUSrc),
        .EX_ALUsel(EX_ALUsel),
        .EX_memwriteen(EX_memwriteen),
        .EX_read1(EX_read1),
        .EX_read2(EX_read2)
    );


    ALU alu (
        .a(EX_read1),
        .b(EX_read2),
        .imm(EX_Immediate),
        .ALUSrc(EX_ALUSrc),
        .sel(EX_ALUsel),
        .out(EX_ALUResult),
        .zero(zero)

    );


    EXregisterMEM EX_R_MEM(
        .clk(clk), 
        .EX_PCplus4(EX_PCplus4), 
        .reset(MEM_reset), 
        .EX_wdat(EX_read2), 
        .EX_wen(EX_wen), 
        .EX_WBSel(EX_WBSel),
        .EX_memwriteen(EX_memwriteen),
        .EX_ALUResult(EX_ALUResult),

        .MEM_Instruction(MEM_Instruction),
        .EX_Instruction(EX_Instruction),
        
        .MEM_wdat(MEM_wdat), 
        .MEM_wen(MEM_wen), 
        .MEM_WBSel(MEM_WBSel), 
        .MEM_memwriteen(MEM_memwriteen), 
        .MEM_PCplus4(MEM_PCplus4),
        .MEM_ALUResult(MEM_ALUResult)
        );



    DataMemory DM (
        .clk(clk),
        .address(MEM_ALUResult),
        .wdat(MEM_wdat),
        .memwriteen(MEM_memwriteen),
        .readD(loadfromMem)
    );

    MEMregisterWB MEM_R_WB(
        .clk(clk), 
        .reset(WB_reset), 
        .MEM_out(loadfromMem), 
        .MEM_wen(MEM_wen), 
        .MEM_WBSel(MEM_WBSel), 
        .MEM_ALUResult(MEM_ALUResult),

        .MEM_Instruction(MEM_Instruction),
        .WB_Instruction(WB_Instruction),


        .WB_readD(WB_readD), 
        .WB_ALUResult(WB_ALUResult),
        .WB_wen(WB_wen), 
        .WB_WBSel(WB_WBSel)
    );



    WriteBackMUX WB (
        .ALUResult(WB_ALUResult),
        .LoadData(WB_readD),
        .WBSel(WB_WBSel),
        .WriteBack(wdat)
    );



endmodule



module Testbench;

    // Clock signal
    reg clk;
    wire [31:0] ID_Instruction, IF_instruction,EX_Instruction,MEM_Instruction,WB_Instruction;

    // Instantiate the processor
    Processor dut (
        .clk(clk),
        .ID_Instruction(ID_Instruction),
        .instruction(IF_instruction),
        .EX_Instruction(EX_Instruction),
        .MEM_Instruction(MEM_Instruction),
        .WB_Instruction(WB_Instruction)
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

    always @(posedge clk) begin
    $display("IF Stage: instr=%h | ID Stage: instr=%h | EX Stage: instr=%h | MEM Stage: instr=%h | WB Stage: instr=%h",
             IF_instruction, ID_Instruction, EX_Instruction, MEM_Instruction, WB_Instruction);
    end


endmodule
