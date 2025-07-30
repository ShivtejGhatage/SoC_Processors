module PC4adder (
    input  [7:0] address,
    output [7:0] IF_PCplus4
);
    assign IF_PCplus4 = address + 8'd8;
endmodule


module ProgramCounter (
    input clk,
    input branch,
    input [7:0] BranchInstr,
    input zero,
    output reg [7:0] address,
    output [7:0] PCplus4
);

    assign PCplus4 = address + 8'd8;

    initial begin
        address <= 8'b0;
    end

    always @(posedge clk) begin
        $display("PC = %h", address);
        if (branch && zero) begin
            address <= BranchInstr;
        end else begin
            address <= address + 8'd8;
        end
    end
endmodule


module InstructionMemory (address, stall, IF_instruction_1, IF_instruction_2);
    input [7:0] address;
    input stall;
    output reg [31:0] IF_instruction_1,IF_instruction_2;
    
    reg [31:0] RAM [0:10]; //Can do 64 tho (6 bits)

    integer i;
    initial begin
        $readmemh("instructions.hex", RAM);
    end


    always @(*) begin
        if (~stall) begin
            IF_instruction_1 = RAM[address[7:2]];  
            IF_instruction_2 = RAM[address[7:2] + 1];
            $display("Now Fetched instruction 1 @ PC=%h: %h", address, IF_instruction_1);
            $display("Now Fetched instruction 2 @ PC=%h: %h", address, IF_instruction_2);
        end 
        else begin
            $display("Stalled at instruction 1 @ PC=%h: %h", address, IF_instruction_1);
            $display("Stalled at instruction 2 @ PC=%h: %h", address, IF_instruction_2);
        end 

        
    end

endmodule





module ControlCenter(clk,instruction, ALUsel, wen, WBSel, ALUSrc, branch, memwriteen, uses_rd,uses_rs1,uses_rs2, rs1,rs2,rd);
    input clk;
    input [31:0] instruction;
    wire [6:0] opcode;
    wire [6:0] f7;
    wire [2:0] f3;
    output reg [3:0] ALUsel;
    // output reg [1:0] dirtchanger; // for dirtying 1. does it dirty or undirty 2. involved with dirtying? Cancel
    output reg wen,WBSel, ALUSrc, branch, memwriteen;
    output reg uses_rs2, uses_rs1, uses_rd; // these work for all instr as of Multicycle
    output reg [4:0] rs1,rs2,rd;
    
    assign opcode = instruction[6:0];
    assign f7 = instruction[31:25];
    assign f3 = instruction[14:12]; 
    
// .rs(ID_Instruction[19:15]),
//         .rt(ID_Instruction[24:20]),
//         .rd(WB_Instruction[11:7]),

    // send for slt sel[3] = 1

    always @(posedge(clk)) begin
        branch = 0; //reset
        ALUSrc = 1;
        memwriteen = 0;
        WBSel = 0;
        wen = 0;
        uses_rd = 0;
        uses_rs1 = 0;
        uses_rs2 = 0;
        rs1 = instruction[19:15];
        rs2 = instruction[24:20];
        rd = instruction[14:12];
        // dirtchanger = 2'b0;

        //ALU stuff
        if (opcode[4:0] == 5'b10011) begin
            ALUsel = {f7[5], f3};
            ALUSrc = opcode[5];     //if 1 then b otherwise immediate
            wen = 1;
            uses_rd = 1;
            uses_rs1 = 1 ;
            uses_rs2 = opcode[5];   //if 1 then b otherwise immediate
            // dirtchanger = 2'b11;
        end

        //Branch stuff
        else if (opcode == 7'b1100011) begin
            branch = 1;
        //  dirtchanger = 2'b00;
            uses_rd = 0;
            uses_rs1 = 1;
            uses_rs2 = 1;
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
            uses_rd = 0;
            uses_rs1 = 1;
            uses_rs2 = 1;
            // dirtchanger = 2'b01;
        end 

        // Load full word and half word
        else if (opcode == 7'b0000011) begin
            WBSel = 1;
            wen = 1;
            ALUSrc = 0;
            ALUsel = 4'b0000;
            uses_rd = 1;
            uses_rs1 = 1;
            uses_rs2 = 0;
        //  dirtchanger = 1;
        end
    end
endmodule



module FIFO (instruction_in_dec, clk, stall_S, instruction_out_dec, full,empty);
    // rs1,rs2,rd,ALUsel,wen,WBSel,ALUSrc,branch,memwriteen, uses rd,rs1,rs2
    // 5  5.  5.  4.    1.   1.    1.      1.      1             1. 1.   1           = 27
    input [117:0] instruction_in_dec;     // no pipeline reg needed here included it inside this
    input stall_S,clk;
    output reg [117:0] instruction_out_dec;
    output full,empty;

    reg [117:0] Queue[0:10];
    reg [3:0] head = 0, tail = 0;
    reg [4:0] count = 0;


    assign full = (count == 11);
    assign empty = (count == 0);

    always @(posedge clk) begin
        if (~full) begin
            Queue[tail] <= instruction_in_dec;
            count <= count+1;
            tail <= (tail == 10) ? 0 : tail+1; //Much more elegant than original bs
        end

        if (~stall_S & ~empty) begin
            instruction_out_dec <= Queue[head];
            count <= count-1;
            head <= (head == 10) ? 0 : head+1;
        end
        
    end


endmodule


module Scheduler (
    input clk,
    input [26:0] instr_to_sched_1,
    input [26:0] instr_to_sched_2,
    output reg [26:0] instr_scheded_1,
    output reg [26:0] instr_scheded_2,
    output reg FIFO_stall

);
    reg issued1,issued2;
    reg Scoreboard [0:31];
    wire uses1_rd, uses1_rs1, uses1_rs2, uses2_rd, uses2_rs1, uses2_rs2;
    wire [4:0] rs1_1, rs2_1, rd_1;
    wire [4:0] rs1_2, rs2_2, rd_2;
    

    wire ready1 = !(uses1_rs1 && Scoreboard[rs1_1]) && !(uses1_rs2 && Scoreboard[rs2_1]);  
    wire ready2 = !(uses2_rs1 && Scoreboard[rs1_2]) && !(uses2_rs2 && Scoreboard[rs2_2]); 

    assign rs1_1 = instr_to_sched_1[26:22];
    assign rs2_1 = instr_to_sched_1[21:17];
    assign rd_1  = instr_to_sched_1[16:12];
    assign uses1_rd = instr_to_sched_1[2];
    assign uses1_rs1 = instr_to_sched_1[1];
    assign uses1_rs2 = instr_to_sched_1[0];


    assign rs1_2 = instr_to_sched_2[26:22];
    assign rs2_2 = instr_to_sched_2[21:17];
    assign rd_2  = instr_to_sched_2[16:12];
    assign uses2_rd = instr_to_sched_2[2];
    assign uses2_rs1 = instr_to_sched_2[1];
    assign uses2_rs2 = instr_to_sched_2[0];
 

    always @(posedge clk) begin
        
        issued1 = 0;
        issued2 = 0;
        
        if (ready1) begin
            instr_scheded_1 <= instr_to_sched_1;
            if (uses1_rd) Scoreboard[rd_1] <= 1;
            issued1 = 1;
        end else begin
            instr_scheded_1 <= 27'b0;  // nop
        end

        if (ready2 && ready1 &&!(uses2_rs1 && (rs1_2 == rd_1)) &&  !(uses2_rs2 && (rs2_2 == rd_1))) begin

            instr_scheded_2 <= instr_to_sched_2;     
            if (uses2_rd)  Scoreboard[rd_2] <= 1;
            issued2 = 1;
        end
        else begin
                instr_scheded_2 <= 27'b0; // nop
        end
        
        FIFO_stall <= !(issued1 || issued2);

    end



endmodule



module RegisterFile (clk,rs,rt,rd,wdat,wen,read1,read2,rs2,rt2,rd2,wdat2,wen2,read1_2,read2_2);
    input clk;
    input [4:0] rs,rt,rd;
    input [4:0] rs2,rt2,rd2;
    input [31:0] wdat,wdat2;
    input wen,wen2;

    output reg [31:0] read1,read2;
    output reg [31:0] read1_2,read2_2;

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
        if (wen2 && rd2 != 0) begin
            Registers[rd2] <= wdat2;          // Its always rd here not like in MIPS
            $display("WRITE: x%0d <= %h (wdat2), wen2=%b, from module at time %t", rd2, wdat2, wen2, $time);

        end
    end

    always @(*) begin
        read1 = Registers[rs];
        read2 = Registers[rt];
        $display("READ: rs=x%0d -> %h, rt=x%0d -> %h", rs, read1, rt, read2);
        read1_2 = Registers[rs2];
        read2_2 = Registers[rt2];
        $display("READ: rs2=x%0d -> %h, rt2=x%0d -> %h", rs2, read1_2, rt2, read2_2);
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




module DataMemory(clk, address,address2, wdat,wdat2, memwriteen,memwriteen2, readD,readD2);
    input clk;
    input [31:0] address,address2;
    input [31:0] wdat,wdat2;
    input memwriteen,memwriteen2;
    output [31:0] readD,readD2;

    reg [31:0] DRAM [63:0];

    integer i;
    initial begin
        for (i = 0; i < 64; i = i + 1)
            DRAM[i] = 32'b0;
    end

    assign readD = DRAM[address[7:2]];
    assign readD2 = DRAM[address2[7:2]];

    always @(posedge clk) begin
        if (memwriteen) begin
            DRAM[address[7:2]] <= wdat;
            $display("Address: %h now has data: %h", address[31:2],wdat);
        end
        if (memwriteen2) begin
            DRAM[address2[7:2]] <= wdat2;
            $display("Address2: %h now has data: %h", address2[31:2],wdat2);
        end
    end
endmodule




module WriteBackMUX(ALUResult, LoadData, WBSel, WriteBack);
    input [31:0] ALUResult, LoadData;
    input WBSel;

    output reg [31:0] WriteBack;

    always @(*) begin
        WriteBack = WBSel ? LoadData : ALUResult;
        //$display("WRITEBACK: wdat=%h to rd=x%0d, wen=%b", wdat, WB_Instruction[11:7], WB_wen);
    end

endmodule




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





module Processor (clk,ID_Instruction,EX_Instruction,instruction,MEM_Instruction,WB_Instruction,
                      ID_Instruction2,EX_Instruction2,instruction2,MEM_Instruction2,WB_Instruction2);
    input clk;
    

    wire branch;
    wire [7:0] BranchInstr,address;
    wire [7:0] BranchInstr2,address2;

    wire [7:0] PCplus4,PCplus42;

    output [31:0] instruction;
    output [31:0] instruction2;

    wire wen,wen2;
    wire [31:0] wdat,read1,read2;
     wire [31:0] wdat2,read1_2,read2_2;

    wire [3:0] ALUsel,ALUsel2;
    wire ALUSrc,ALUSrc2;
    wire[31:0] immExtended,immExtended2;
    wire [31:0] ALUResult,ALUResult2;

    wire memwriteen,memwriteen2;

    wire [31:0] loadfromMem,loadfromMem2;

    wire WBSel,WBSel2;

    wire zero,zero2;



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


    //part 2
    output [31:0] ID_Instruction2;
    wire [7:0] ID_PCplus42;

    wire [7:0] EX_PCplus42;
    wire [31:0] EX_Immediate2;
    wire [7:0] EX_BranchAddr2;
    output [31:0] EX_Instruction2;
    wire [31:0] EX_read12;
    wire [31:0] EX_read22;
    wire [31:0] EX_ALUResult2;
    wire [31:0] WB_readD2;
    wire [31:0] WB_ALUResult2;


    wire EX_ALUSrc2,EX_wen2,EX_WBSel2,EX_branch2,EX_memwriteen2;
    wire [3:0] EX_ALUsel2;

    wire [7:0] MEM_PCplus42;
    wire MEM_wen2,MEM_WBSel2,MEM_branch2,MEM_memwriteen2;
    wire [31:0] MEM_ALUResult2;
    wire [31:0] MEM_wdat2;
    output [31:0] WB_Instruction2;
    output [31:0] MEM_Instruction2;

    wire stall_ID=0,stall_EX=0,stall_MEM=0,stall_WB=0,stall_FIFO=0;

    wire stall =0;

    wire [4:0] rs1,rs2,rd,rs12,rs22,rd2;
    wire uses_rs1,uses_rs2,uses_rd,uses_rs12,uses_rs22,uses_rd2;

    wire [117:0] instrFO;

    wire full,empty;



    // ---------- Instruction 1 ----------
    wire [31:0] FO_immExtended   = instrFO[117:86];
    wire [4:0]  FO_rs1           = instrFO[85:81];
    wire [4:0]  FO_rs2           = instrFO[80:76];
    wire [4:0]  FO_rd            = instrFO[75:71];
    wire [3:0]  FO_ALUsel        = instrFO[70:67];
    wire        FO_wen           = instrFO[66];
    wire        FO_WBSel         = instrFO[65];
    wire        FO_ALUSrc        = instrFO[64];
    wire        FO_branch        = instrFO[63];
    wire        FO_memwriteen    = instrFO[62];
    wire        FO_uses_rd       = instrFO[61];
    wire        FO_uses_rs1      = instrFO[60];
    wire        FO_uses_rs2      = instrFO[59];

    // ---------- Instruction 2 ----------
    wire [31:0] FO_immExtended2  = instrFO[58:27];
    wire [4:0]  FO_rs12          = instrFO[26:22];
    wire [4:0]  FO_rs22          = instrFO[21:17];
    wire [4:0]  FO_rd2           = instrFO[16:12];
    wire [3:0]  FO_ALUsel2       = instrFO[11:8];
    wire        FO_wen2          = instrFO[7];
    wire        FO_WBSel2        = instrFO[6];
    wire        FO_ALUSrc2       = instrFO[5];
    wire        FO_branch2       = instrFO[4];
    wire        FO_memwriteen2   = instrFO[3];
    wire        FO_uses_rd2      = instrFO[2];
    wire        FO_uses_rs12     = instrFO[1];
    wire        FO_uses_rs22     = instrFO[0];



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
        .IF_instruction_1(instruction),
        .stall(stall),
        .IF_instruction_2(instruction2)
    );

    IFregisterID IF_R_ID(
        .IF_instruction(instruction),
        .clk(clk),
        .reset(ID_reset),
        .IF_PCplus4(PCplus4),
        .ID_Instruction(ID_Instruction),
        .ID_PCplus4(ID_PCplus4)
    );

    IFregisterID IF_R_ID2(
        .IF_instruction(instruction2),
        .clk(clk),
        .reset(ID_reset2),
        .IF_PCplus4(PCplus42),
        .ID_Instruction(ID_Instruction2),
        .ID_PCplus4(ID_PCplus42)
    );

    //instruction, ALUsel, wen, WBSel, ALUSrc, branch, memwriteen, uses_rd,uses_rs1,uses_rs2
    ControlCenter Ctrl (
        .clk(clk),
        .instruction(ID_Instruction),
        // .opcode(ID_Instruction[6:0]),
        // .f7(ID_Instruction[31:25]),
        // .f3(ID_Instruction[14:12]),
        .ALUsel(ALUsel),
        .memwriteen(memwriteen),
        .wen(wen),
        .WBSel(WBSel), 
        .branch(branch), 
        .ALUSrc(ALUSrc),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd)
    );
    ControlCenter Atl (
        .clk(clk),
        .instruction(ID_Instruction),
        // .opcode(ID_Instruction2[6:0]),
        // .f7(ID_Instruction2[31:25]),
        // .f3(ID_Instruction2[14:12]),
        .ALUsel(ALUsel2),
        .memwriteen(memwriteen2),
        .wen(wen2),
        .WBSel(WBSel2), 
        .branch(branch2), 
        .ALUSrc(ALUSrc2),
        .rs1(rs12),
        .rs2(rs22),
        .rd(rd2)
    );

        



    ImmediateGenerator ImG (
        .instr(ID_Instruction),
        .immediate(immExtended)
    );
    ImmediateGenerator ImG2 (
        .instr(ID_Instruction2),
        .immediate(immExtended2)
    );

    ImmediatePC4adder IPC4(
        .immediate(immExtended),
        .PCplus4(ID_PCplus4),
        .BranchAddr(BranchInstr)
    );

    ImmediatePC4adder IPC42(
        .immediate(immExtended2),
        .PCplus4(ID_PCplus42),
        .BranchAddr(BranchInstr2)
    );



    FIFO f(
        .instruction_in_dec({immExtended,rs1,rs2,rd,ALUsel,wen,WBSel,ALUSrc,branch,memwriteen, uses_rd,uses_rs1,uses_rs2,
                            immExtended2,rs12,rs22,rd2,ALUsel2,wen2,WBSel2,ALUSrc2,branch2,memwriteen2, uses_rd2,uses_rs12,uses_rs22
        }), 
        .clk(clk), 
        .stall_S(stall_FIFO), 
        .instruction_out_dec(instrFO), 
        .full(full),
        .empty(empty));
    // rs1,rs2,rd,ALUsel,wen,WBSel,ALUSrc,branch,memwriteen, uses rd,rs1,rs2
    // 5  5.  5.  4.    1.   1.    1.      1.      1             1. 1.   1           = 27
    //input [53:0] instruction_in_dec;     // no pipeline reg needed here included it inside this


    RegisterFile RF (
        .clk(clk),
        .rs(FO_rs1),
        .rt(FO_rs2),
        .rd(FO_rd),
        .wdat(wdat),
        .wen(WB_wen),
        .read1(read1),
        .read2(read2),

        .rs2(FO_rs12),
        .rt2(FO_rs22),
        .rd2(FO_rd2),
        .wdat2(wdat2),
        .wen2(WB_wen2),
        .read1_2(read1_2),
        .read2_2(read2_2)
    );


    IDregisterEX ID_R_EX(
        .clk(clk),
        .reset(EX_reset),

        .ID_Instruction(ID_Instruction),
        .EX_Instruction(EX_Instruction),

        .ID_PCplus4(ID_PCplus4),
        .EX_PCplus4(EX_PCplus4),
        .ID_Immediate(FO_immExtended),
        .EX_Immediate(EX_Immediate),
        .ID_BranchAddr(BranchInstr),
        .EX_BranchAddr(EX_BranchAddr),
        .ALUsel(FO_ALUsel),
        .wen(FO_wen),
        .WBSel(FO_WBSel),
        .branch(FO_branch), 
        .ALUSrc(FO_ALUSrc),
        .memwriteen(FO_memwriteen),
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

    
    
    IDregisterEX ID_R_EX_2(
        .clk(clk),
        .reset(EX_reset),

        .ID_Instruction(ID_Instruction2),
        .EX_Instruction(EX_Instruction2),

        .ID_PCplus4(ID_PCplus42),
        .EX_PCplus4(EX_PCplus42),
        .ID_Immediate(FO_immExtended2),
        .EX_Immediate(EX_Immediate2),
        .ID_BranchAddr(BranchInstr2),
        .EX_BranchAddr(EX_BranchAddr2),
        .ALUsel(FO_ALUsel2),
        .wen(FO_wen2),
        .WBSel(FO_WBSel2),
        .branch(FO_branch2), 
        .ALUSrc(FO_ALUSrc2),
        .memwriteen(FO_memwriteen2),
        .ID_read1(read1_2),
        .ID_read2(read2_2),

        .EX_wen(EX_wen2),
        .EX_WBSel(EX_WBSel2),
        .EX_branch(EX_branch2), 
        .EX_ALUSrc(EX_ALUSrc2),
        .EX_ALUsel(EX_ALUsel2),
        .EX_memwriteen(EX_memwriteen2),
        .EX_read1(EX_read12),
        .EX_read2(EX_read22)
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

    ALU alu2 (
        .a(EX_read12),
        .b(EX_read22),
        .imm(EX_Immediate2),
        .ALUSrc(EX_ALUSrc2),
        .sel(EX_ALUsel2),
        .out(EX_ALUResult2),
        .zero(zero2)

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

    EXregisterMEM EX_R_MEM_2(
        .clk(clk), 
        .EX_PCplus4(EX_PCplus42), 
        .reset(MEM_reset2), 
        .EX_wdat(EX_read22), 
        .EX_wen(EX_wen2), 
        .EX_WBSel(EX_WBSel2),
        .EX_memwriteen(EX_memwriteen2),
        .EX_ALUResult(EX_ALUResult2),

        .MEM_Instruction(MEM_Instruction2),
        .EX_Instruction(EX_Instruction2),
        
        .MEM_wdat(MEM_wdat2), 
        .MEM_wen(MEM_wen2), 
        .MEM_WBSel(MEM_WBSel2), 
        .MEM_memwriteen(MEM_memwriteen2), 
        .MEM_PCplus4(MEM_PCplus42),
        .MEM_ALUResult(MEM_ALUResult2)
        );




    DataMemory DM (
        .clk(clk),
        .address(MEM_ALUResult),
        .wdat(MEM_wdat),
        .memwriteen(MEM_memwriteen),
        .readD(loadfromMem),

        .address2(MEM_ALUResult2),
        .wdat2(MEM_wdat2),
        .memwriteen2(MEM_memwriteen2),
        .readD2(loadfromMem2)
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

    MEMregisterWB MEM_R_WB_2(
        .clk(clk), 
        .reset(WB_reset2), 
        .MEM_out(loadfromMem2), 
        .MEM_wen(MEM_wen2), 
        .MEM_WBSel(MEM_WBSel2), 
        .MEM_ALUResult(MEM_ALUResult2),

        .MEM_Instruction(MEM_Instruction2),
        .WB_Instruction(WB_Instruction2),


        .WB_readD(WB_readD2), 
        .WB_ALUResult(WB_ALUResult2),
        .WB_wen(WB_wen2), 
        .WB_WBSel(WB_WBSel2)
    );



    WriteBackMUX WB (
        .ALUResult(WB_ALUResult),
        .LoadData(WB_readD),
        .WBSel(WB_WBSel),
        .WriteBack(wdat)
    );

    WriteBackMUX WB2 (
        .ALUResult(WB_ALUResult2),
        .LoadData(WB_readD2),
        .WBSel(WB_WBSel2),
        .WriteBack(wdat2)
    );



endmodule


module Testbench;

    // Clock signal
    reg clk;
    wire [31:0] ID_Instruction, IF_instruction,EX_Instruction,MEM_Instruction,WB_Instruction;
    wire [31:0] ID_Instruction2, IF_instruction2,EX_Instruction2,MEM_Instruction2,WB_Instruction2;

    // Instantiate the processor
    Processor dut (
        .clk(clk),
        .ID_Instruction(ID_Instruction),
        .instruction(IF_instruction),
        .EX_Instruction(EX_Instruction),
        .MEM_Instruction(MEM_Instruction),
        .WB_Instruction(WB_Instruction),
        .ID_Instruction2(ID_Instruction2),
        .instruction2(IF_instruction2),
        .EX_Instruction2(EX_Instruction2),
        .MEM_Instruction2(MEM_Instruction2),
        .WB_Instruction2(WB_Instruction2)
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

        #40;  // Run for 1000ns (100 clock cycles)
        $display("Simulation done.");
        $finish;
    end

    always @(posedge clk) begin
    $display("First  : IF Stage: instr=%h | ID Stage: instr=%h | EX Stage: instr=%h | MEM Stage: instr=%h | WB Stage: instr=%h",
             IF_instruction, ID_Instruction, EX_Instruction, MEM_Instruction, WB_Instruction);
    $display("Second : IF Stage: instr=%h | ID Stage: instr=%h | EX Stage: instr=%h | MEM Stage: instr=%h | WB Stage: instr=%h",
             IF_instruction2, ID_Instruction2, EX_Instruction2, MEM_Instruction2, WB_Instruction2);
    end


endmodule
