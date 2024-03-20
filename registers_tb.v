`timescale 1ns/1ns
`include "registers.v"

module register_file_tb;


reg clk, reset, RegWrite;
reg [4:0] Rs1, Rs2, Rd;
reg [15:0] Write_data;
wire [15:0] Read_data1, Read_data2;


register_file rf_unit (
    .clk(clk),
    .reset(reset),
    .Rs1(Rs1),
    .Rs2(Rs2),
    .Rd(Rd),
    .Write_data(Write_data),
    .RegWrite(RegWrite),
    .Read_data1(Read_data1),
    .Read_data2(Read_data2)
);


initial begin
    clk = 0; 
    forever #5 clk = ~clk; 
end


initial begin

    $dumpfile("register_file_tb.vcd");
    $dumpvars(0, register_file_tb);

    reset = 1;
    #10; 
    reset = 0;


    RegWrite = 1;
    Rd = 2;
    Write_data = 16'h1234;
    #10;
   
    Rd = 3;
    Write_data = 16'h5678; 
    #10;


    RegWrite = 0; 
    Rs1 = 2;
    Rs2 = 3;
    #10;

    $finish; 
end

// Monitor results
initial begin
    $monitor("Time: %t | Read_data1 = %h | Read_data2 = %h", $time, Read_data1, Read_data2);
end

endmodule
