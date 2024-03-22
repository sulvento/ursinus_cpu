`timescale 1ns/1ns
`include "memory_correction_register.v"

module memory_correction_register_tb;

  reg clk;
  reg reset;
  reg error_correction_event;
  wire [15:0] correction_count_out;

  memory_correction_register dut (
    .clk(clk),
    .reset(reset),
    .error_correction_event(error_correction_event),
    .correction_count_out(correction_count_out)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk; 
  end

  initial begin

    $dumpfile("memory_correction_register_tb.vcd");
    $dumpvars(0, memory_correction_register_tb);

    reset = 1;
    #10;
    reset = 0; 

    // Simulate a few correction events
    error_correction_event = 1;
    #10;
    error_correction_event = 0;
    #15; 
    error_correction_event = 1;
    #20; 
    error_correction_event = 0; 

    // Longer pause
    #50;

    // More infrequent events
    error_correction_event = 1; 
    #30;
    error_correction_event = 0;

    $finish; 
  end

  // Monitor signals
  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b, error_correction_event: %b, correction_count_out: %d",
              $time, clk, reset, error_correction_event, correction_count_out); 
  end

endmodule 
