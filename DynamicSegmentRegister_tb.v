`timescale 1ns/1ns
`include "DynamicSegmentRegister.v"

module dynamic_segment_register_tb;

  reg clk;
  reg reset;
  reg load_dsr;
  reg [15:0] dsr_data_in;
  wire [15:0] dsr_data_out;

  dynamic_segment_register dut (
    .clk(clk),
    .reset(reset),
    .load_dsr(load_dsr),
    .dsr_data_in(dsr_data_in),
    .dsr_data_out(dsr_data_out)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin

    $dumpfile("dynamic_segment_register_tb.vcd");
    $dumpvars(0, dynamic_segment_register_tb);

    reset = 1; 
    load_dsr = 0;
    #10; 
    reset = 0; 

    load_dsr = 1;

    dsr_data_in = 16'h3000; 
    #10; 
    load_dsr = 0; 

    dsr_data_in = 16'h5800; 
    load_dsr = 1;
    #10;
    load_dsr = 0; 

    #50;

    $finish; 
  end


  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b, load_dsr: %b, dsr_data_in: %h, dsr_data_out: %h",
              $time, clk, reset, load_dsr, dsr_data_in, dsr_data_out); 
  end

endmodule 
