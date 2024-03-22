`include "statseg.v"

module testbench_statseg;
    reg clk = 0;
    reg reset, loadsig;
    reg [19:0] data_in;
    wire [19:0] data_out;

statseg SS(
    .clk(clk),
    .reset(reset),
    .loadsig(loadsig),
    .data_in(data_in),
    .data_out(data_out)
);

always #5 clk = ~clk;

initial begin
    $dumpfile("testbench_statseg.vcd");
    $dumpvars(1,SS);

    reset = 1;
    #5;
    reset = 0;
    loadsig = 1;
    data_in = 20'hAD1;
    #5;

    $finish;

end



endmodule
