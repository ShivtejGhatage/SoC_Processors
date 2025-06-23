module RegisterFile (clk,rs,rt,rd,wdat,wen, read1,read2);
    input clk;
    input [5:0] rs,rt,rd;
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