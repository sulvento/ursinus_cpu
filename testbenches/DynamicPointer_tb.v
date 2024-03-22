`timescale 1ns/1ns
`include "DynamicPointer.v"

module dynamic_pointer_tb;

  reg clk;
  reg reset;
  reg load_dp;
  reg inc_dp;
  reg write_dp;
  reg [15:0] dp_data_in;
  wire [15:0] dp_data_out;

  dynamic_pointer dut (
    .clk(clk),
    .reset(reset),
    .load_dp(load_dp),
    .inc_dp(inc_dp),
    .write_dp(write_dp),
    .dp_data_in(dp_data_in),
    .dp_data_out(dp_data_out)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin

    $dumpfile("dynamic_pointer_tb.vcd");
    $dumpvars(0, dynamic_pointer_tb);

    
    reset = 1;
    #10;
    reset = 0;

    load_dp = 1;
    dp_data_in = 16'h3800; 
    #10;
    load_dp = 0;

    inc_dp = 1;
    #10;
    inc_dp = 0;
    #10;
    inc_dp = 1;
    #10;
    inc_dp = 0;

    write_dp = 1;
    dp_data_in = 16'h5000;
    #10;
    write_dp = 0;

    #10;
    inc_dp = 1;
    #10; 
    inc_dp = 0; 

    #30;  
    
    $finish; 
  end

  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b, load_dp: %b, inc_dp: %b, write_dp: %b, dp_data_in: %h, dp_data_out: %h",
              $time, clk, reset, load_dp, inc_dp, write_dp, dp_data_in, dp_data_out); 
  end

endmodule 
