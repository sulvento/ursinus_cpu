`include "ipreg.v"

module testbench_ipreg;

reg clk = 0;
reg reset, inc;
reg [19:0] data_in;
wire [19:0] data_out;


ipreg IP(
    .clk(clk),
    .reset(reset),
    .inc(inc),
    .data_in(data_in),
    .data_out(data_out)
);

always #5 clk = ~clk;

initial begin
    $dumpfile("testbench_ipreg.vcd");
    $dumpvars(1,IP);

    reset = 1;
    #5;
    reset = 0;
    inc = 1;
    data_in = 0;
    #5;
    inc = 0;
    #5;
    inc = 1;
    #5;
    inc = 0;
    #5;
    inc = 1;
    #20;
    $finish;

end



endmodule
