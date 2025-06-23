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