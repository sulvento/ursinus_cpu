`timescale 1ns/1ns
`include "memory_access_register.v"

module memory_access_register_tb;

  reg clk;
  reg reset;
  reg count_access;
  wire [15:0] access_count_out;

  memory_access_register dut (
    .clk(clk),
    .reset(reset),
    .count_access(count_access),
    .access_count_out(access_count_out)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin

    $dumpfile("memory_access_register_tb.vcd");
    $dumpvars(0, memory_access_register_tb);

    reset = 1;
    #10;
    reset = 0; 

    count_access = 1;
    #10;
    count_access = 0;
    #15; 
    count_access = 1;
    #10; 
    count_access = 0; 
    #5;
    count_access = 1;
    #10;
    #60;

    $finish; 
  end

  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b, count_access: %b, access_count_out: %d",
              $time, clk, reset, count_access, access_count_out); 
  end

endmodule 
