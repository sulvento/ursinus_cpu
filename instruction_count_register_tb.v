`timescale 1ns/1ns
`include "instruction_count_register.v"

module instruction_count_register_tb;

  reg clk;
  reg reset;
  reg update_count;
  wire [15:0] count_out;

  instruction_count_register dut (
    .clk(clk),
    .reset(reset),
    .update_count(update_count),
    .count_out(count_out)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    $dumpfile("instruction_count_register_tb.vcd");
    $dumpvars(0, instruction_count_register_tb);
    reset = 1;
    #10;
    reset = 0; 

    update_count = 1;
    #10;
    update_count = 0;
    #10;
    update_count = 1;
    #20; 
    update_count = 0; 

    #50;

    update_count = 1;
    #10; 
    update_count = 0;

    $finish; 
  end

  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b,  update_count: %b, count_out: %d",
              $time, clk, reset, update_count, count_out); 
  end

endmodule 
