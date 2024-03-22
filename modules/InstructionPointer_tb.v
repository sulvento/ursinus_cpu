`timescale 1ns/1ns
`include "InstructionPointer.v"

module instruction_pointer_tb;

  reg clk;
  reg reset;
  reg load_ip;
  reg inc_ip;
  reg [15:0] ip_data_in;
  wire [15:0] ip_data_out;

  instruction_pointer dut (
    .clk(clk),
    .reset(reset),
    .load_ip(load_ip),
    .inc_ip(inc_ip),
    .ip_data_in(ip_data_in),
    .ip_data_out(ip_data_out)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk; 
  end

  initial begin

    $dumpfile("instruction_pointer_tb.vcd");
    $dumpvars(0, instruction_pointer_tb);
    reset = 1; 
    #10; 
    reset = 0; 

    load_ip = 1;
    ip_data_in = 16'h1000; 
    #10;
    load_ip = 0;

    inc_ip = 1; 
    #10;
    inc_ip = 0; 
    #10; 
    inc_ip = 1;
    #10; 
    inc_ip = 0;

    load_ip = 1;
    ip_data_in = 16'h3500; 
    #10;
    load_ip = 0;

    #10; 
    inc_ip = 1; 
    #10; 
    inc_ip = 0;

    #30;  

    $finish; 
  end

  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b, load_ip: %b, inc_ip: %b, ip_data_in: %h, ip_data_out: %h",
              $time, clk, reset, load_ip, inc_ip, ip_data_in, ip_data_out); 
  end

endmodule 
