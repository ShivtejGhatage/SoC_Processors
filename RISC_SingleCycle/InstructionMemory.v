module InstructionMemory (address, instruction);
    input [5:0] address;
    output [31:0] instruction;
    
    reg [31:0] RAM [63:0];

    initial begin
    $readmemh("instructions.dat", RAM);
    end

    assign instruction = RAM[address];


endmodule