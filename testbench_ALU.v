`include "alu.v"

module testbench_ALU;

reg [13:0] instruction;
reg [19:0] A; 
reg [19:0] B;
wire [19:0] result;
wire carry_out;


ALU tb_alu(
    .instruction(instruction),
    .A(A),
    .B(B),
    .result(result)
);

initial begin
    $dumpfile("testbench_ALU.vcd");
    $dumpvars(1,tb_alu);
    $monitor ("instruction: 0b%0b - A: 0b%0b - B: 0b%0b - result: 0b%0b", 
    instruction, A, B, result);

    instruction = 13'h13A;

    A = 20'b10101010;
    B = 20'b11;
    #10;



    $finish;


end


endmodule