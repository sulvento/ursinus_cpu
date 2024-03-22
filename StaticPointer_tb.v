`timescale 1ns/1ns
`include "StaticPointer.v"

module static_pointer_tb;

  reg clk;
  reg reset;
  reg load_sp;
  reg inc_sp;
  reg write_sp;
  reg [15:0] sp_data_in;
  wire [15:0] sp_data_out;
  wire invalid_write;

  static_pointer dut (
    .clk(clk),
    .reset(reset),
    .load_sp(load_sp),
    .inc_sp(inc_sp),
    .write_sp(write_sp),
    .sp_data_in(sp_data_in),
    .sp_data_out(sp_data_out),
    .invalid_write(invalid_write)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    $dumpfile("static_pointer_tb.vcd");
    $dumpvars(0, static_pointer_tb);

    reset = 1;
    #10;
    reset = 0;

    load_sp = 1;
    sp_data_in = 16'h2800; 
    #10;
    load_sp = 0;

    inc_sp = 1;
    #10;
    inc_sp = 0;

    write_sp = 1;
    sp_data_in = 16'hABCD; 
    #10;
    write_sp = 0;

    load_sp = 1;
    sp_data_in = 16'h3200;
    #10;
    load_sp = 0;

    #50;

    $finish; 
  end

  always @(posedge clk) begin
    $display("Time: %d, clk: %b, reset: %b, load_sp: %b, inc_sp: %b, write_sp: %b, sp_data_in: %h, sp_data_out: %h, invalid_write: %b",
              $time, clk, reset, load_sp, inc_sp, write_sp, sp_data_in, sp_data_out, invalid_write); 
  end

endmodule 
