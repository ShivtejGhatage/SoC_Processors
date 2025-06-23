module adder(a,b,sum,cout);
    input [31:0] a;
    input [31:0] b;
    output [31:0] sum;
    output cout;
    wire [32:0] x;

    assign x = a + b;
    assign sum = x[31:0];
    assign cout = x[32];


endmodule;

module adder_testbench;
    reg [31:0] a,b;
    wire [31:0] sum;
    wire cout;

    adder uut(
        .a(a),
        .b(b),
        .sum(sum),
        .cout(cout)
    );

    initial begin
        $display("Time\t\t a\t\t\t b\t\t\t sum\t\t\t cout");
        $display("---------------------------------------------------------------");

        // Test case 1: Small values
        a = 32'h00000001;
        b = 32'h00000001;
        #10;
        $display("%0t\t %h\t %h\t %h\t %b", $time, a, b, sum, cout);

        // Test case 2: Mid-range values
        a = 32'h0000FFFF;
        b = 32'h00000001;
        #10;
        $display("%0t\t %h\t %h\t %h\t %b", $time, a, b, sum, cout);

        // Test case 3: Overflow case
        a = 32'hFFFFFFFF;
        b = 32'h00000001;
        #10;
        $display("%0t\t %h\t %h\t %h\t %b", $time, a, b, sum, cout);

        // Test case 4: Both max
        a = 32'hFFFFFFFF;
        b = 32'hFFFFFFFF;
        #10;
        $display("%0t\t %h\t %h\t %h\t %b", $time, a, b, sum, cout);

        $finish;

    end

endmodule; 
