module ALU ( a, b, sel, out);
    input [31:0] a;
    input [31:0] b;
    input [2:0] sel;
    output reg [31:0] out;
    wire [31:0] bornotb, sum;
    wire less;

    assign bornotb = sel[2] ? ~b : b;
    assign sum = a + bornotb + sel[2];

    always @ (a or b or sel)
    begin
        case (sel[1:0])
            2'b00 : out = a & bornotb;
            2'b01 : out = a | bornotb;
            2'b10 : out = sum;
            2'b11 : out = {31'b0, less};

            default : out = 32'b0;

        endcase
    end
endmodule