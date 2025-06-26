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





module ControlCenter(instruction, ALUsel, wen, WBSel, ALUSrc, branch, memwriteen, dirtchanger);
    input [31:0] instruction;
    wire [6:0] opcode;
    wire [6:0] f7;
    wire [2:0] f3;
    output reg [3:0] ALUsel;
    output reg dirtchanger; // for dirtying
    output reg wen,WBSel, ALUSrc, branch, memwriteen;
    
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
        dirtchanger = 0;

        //ALU stuff
        if (opcode[4:0] == 5'b10011) begin
            ALUsel = {f7[5], f3};
            ALUSrc = opcode[5];     //if 1 then b otherwise immediate
            wen = 1;
            dirtchanger = 1;
        end

        //Branch stuff
        else if (opcode == 7'b1100011) begin
            branch = 1;
         dirtchanger = 0;
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
         dirtchanger = 1;
        end 

        // Load full word and half word
        else if (opcode == 7'b0000011) begin
            WBSel = 1;
            wen = 1;
            ALUSrc = 0;
            ALUsel = 4'b0000;
         dirtchanger = 1;
        end
    end
endmodule



module FIFO (instruction_in_dec, clk, stall_S, instruction_out_dec, full,empty);
    // rs,rt,rd,ALUsel,wen,WBSel,ALUSrc,branch,memwriteen,dirtchanger
    // 5  5.  5.  4.    1.   1.    1.      1.      1          1.      = 25
    input [24:0] instruction_in_dec;     // no pipeline reg needed here included it inside this
    input stall_S,clk;
    output reg [24:0] instruction_out_dec;
    output full,empty;

    reg [24:0] Queue[0:10];
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
    input [24:0] instr_to_shed_1,
    input [24:0] instr_to_shed_2,
    output [24:0] instr_sheded_1,
    output [24:0] instr_sheded_2
)

    reg Scoreboard [0:31];
    wire dirtchanger1,dirtchanger2,rd1,rd2;
    assign dirtchanger1 = instr_to_shed_1[0];
    assign dirtchanger2 = instr_to_shed_2[0];
    assign rd1 = instr_to_shed_1[21:17];
    assign rd2 = instr_to_shed_2[21:17];

    always @(posedge clk) begin
        if(dirtchanger1) begin
            Scoreboard[rd1] = 
        end

    end




endmodule









