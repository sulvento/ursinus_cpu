`timescale 1ns/1ns
`include "StaticSegmentRegister.v"

module static_segment_register_tb;

  reg clk;
  reg reset;
  reg load_ssr;
  reg [15:0] ssr_data_in;
  wire [15:0] ssr_data_out;


  static_segment_register dut (
    .clk(clk),
    .reset(reset),
    .load_ssr(load_ssr),
    .ssr_data_in(ssr_data_in),
    .ssr_data_out(ssr_data_out)
  );


  initial begin
    clk = 0;
    forever #5 clk = ~clk; 
  end


  initial begin

    $dumpfile("static_segment_register_tb.vcd");
    $dumpvars(0, static_segment_register_tb);

    reset = 1;
    load_ssr = 0;
    #10; 
    reset = 0; 


    load_ssr = 1;
    ssr_data_in = 16'h2000; 
    #10; 
    load_ssr = 0;


    ssr_data_in = 16'h3500; 
    load_ssr = 1;
    #10;
    load_ssr = 0;

    #50;

    $finish; 
  end


  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b, load_ssr: %b, ssr_data_in: %h, ssr_data_out: %h",
              $time, clk, reset, load_ssr, ssr_data_in, ssr_data_out); 
  end

endmodule 
