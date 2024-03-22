`timescale 1ns/1ns
`include "InstructionSegmentRegister.v"

module instruction_segment_register_tb;

  reg clk;
  reg reset;
  reg load_isr;
  reg [15:0] isr_data_in;
  wire [15:0] isr_data_out;


  instruction_segment_register dut (
    .clk(clk),
    .reset(reset),
    .load_isr(load_isr),
    .isr_data_in(isr_data_in),
    .isr_data_out(isr_data_out)
  );


  initial begin
    clk = 0;
    forever #5 clk = ~clk; 
  end


  initial begin

    $dumpfile("instruction_segment_register_tb.vcd");
    $dumpvars(0, instruction_segment_register_tb);

    reset = 1;
    load_isr = 0;
    #10; 
    reset = 0; 


    load_isr = 1;
    isr_data_in = 16'hABCD;
    #10; 
    load_isr = 0;


    isr_data_in = 16'h1234;
    #10;


    load_isr = 1;
    isr_data_in = 16'h5678; 
    #10;
    load_isr = 0;

    #50;

    $finish; 
  end


  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b, load_isr: %b, isr_data_in: %h, isr_data_out: %h",
              $time, clk, reset, load_isr, isr_data_in, isr_data_out); 
  end

endmodule 
