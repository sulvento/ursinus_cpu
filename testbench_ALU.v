`include "alu.v"

module testbench_ALU;

reg [13:0] instruction;
reg [19:0] A; 
reg [19:0] B;
reg clk = 0;
wire [19:0] result;
reg cin;
wire carry_out;



ALU tb_alu(
    .instruction(instruction),
    .A(A),
    .B(B),
    .result(result),
    .clk(clk),
    .cin(cin),
    .carry_out(carry_out)
);

always #5 clk = ~clk;

initial begin
    cin <= 1;
    A <= 20'b101010101;
    B <= 20'b1111;


    $dumpfile("testbench_ALU.vcd");
    $dumpvars(1,tb_alu);
    $monitor ("instruction: 0b%0b - A: 0b%0b - B: 0b%0b - result: 0b%0b - clk: 0b%0b, cin: 0b%0b, carry_out: 0b%0b", 
    instruction, A, B, result, clk, cin, carry_out);
    
    instruction = 13'h18E;
    #20;

    $finish;


end


endmodule
