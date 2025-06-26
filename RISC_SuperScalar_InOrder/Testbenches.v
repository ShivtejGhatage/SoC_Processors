
module IM_tb (
);

    reg clk;
    wire [7:0] address;
    wire [31:0] IF_instruction_1,IF_instruction_2;

    InstructionMemory DUT(
        .address(address),
        .IF_instruction_1(IF_instruction_1),
        .IF_instruction_2(IF_instruction_2)
    );

    ProgramCounter PC (
        .clk(clk),
        .address(address)
    );

    initial begin 
        $display("Testbenching : ");
        clk = 0;


        repeat (8) begin
            #10;
            clk = ~clk;
        end

        $display("Testbench finished.");
        $finish;

    end

endmodule

