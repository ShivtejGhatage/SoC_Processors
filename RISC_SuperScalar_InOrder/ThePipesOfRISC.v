module ProgramCounter (
    input clk,
    input stall,
    output reg [7:0] address
);
    initial begin
        address <= 8'b0;
    end

    always @(posedge clk) begin
        if (~stall) begin
            address <= address + 8'b00000100;
        end
        else begin
            address <= address;
        end
    end

endmodule

module InstructionMemory (address, stall, IF_instruction_1, IF_instruction_2);
    input [7:0] address;
    input stall;
    output reg [31:0] IF_instruction_1,IF_instruction_2;
    
    reg [31:0] RAM [0:10]; //Can do 64 tho (6 bits)

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





module ControlCenter(instruction, ALUsel, wen, WBSel, ALUSrc, branch, memwriteen, uses_rd,uses_rs1,uses_rs2);
    input [31:0] instruction;
    wire [6:0] opcode;
    wire [6:0] f7;
    wire [2:0] f3;
    output reg [3:0] ALUsel;
    // output reg [1:0] dirtchanger; // for dirtying 1. does it dirty or undirty 2. involved with dirtying? Cancel
    output reg wen,WBSel, ALUSrc, branch, memwriteen;
    output reg uses_rs2, uses_rs1, uses_rd; // these work for all instr as of Multicycle
    
    assign opcode = instruction[6:0];
    assign f7 = instruction[31:25];
    assign f3 = instruction[14:12]; 
    

    // send for slt sel[3] = 1

    always @(*) begin
        branch = 0; //reset
        ALUSrc = 1;
        memwriteen = 0;
        WBSel = 0;
        wen = 0;
        uses_rd = 0;
        uses_rs1 = 0;
        uses_rs2 = 0;
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
    input [26:0] instruction_in_dec;     // no pipeline reg needed here included it inside this
    input stall_S,clk;
    output reg [26:0] instruction_out_dec;
    output full,empty;

    reg [26:0] Queue[0:10];
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
        FIFO_stall <= 0;
        
        if (ready1) begin
            instr_scheded_1 <= instr_to_sched_1;
            if (uses1_rd) Scoreboard[rd_1] <= 1;
            
        end else begin
            instr_scheded_1 <= 27'b0;  // nop
        end

        if (ready2 && ready1 &&!(uses2_rs1 && (rs1_2 == rd_1)) &&  !(uses2_rs2 && (rs2_2 == rd_1))) begin

            instr_scheded_2 <= instr_to_sched_2;     
            if (uses2_rd)  Scoreboard[rd_2] <= 1;
            
        end
        else begin
                instr_scheded_2 <= 27'b0; // nop
        end
        
    end




endmodule









