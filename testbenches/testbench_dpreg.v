`include "dpreg.v"

module testbench_dpreg;

reg clk = 0;
reg reset, inc, writesig, readsig;
reg [19:0] data_in;
wire [19:0] data_out;

dynamic_pointer dpreg(
    .clk(clk),
    .reset(reset),
    .inc(inc),
    .writesig(writesig),
    .readsig(readsig),
    .data_in(data_in),
    .data_out(data_out)
);

always #5 clk = ~clk;

initial begin
    $dumpfile("testbench_dpreg.vcd");
    $dumpvars(1,dpreg);

    reset = 1;
    #5;
    reset = 0;
    writesig = 1;
    data_in = 20'b0;
    #5;
    writesig = 0;
    readsig = 1;
    #5; 
    readsig = 0;
    inc = 1;
    #5;
    readsig = 1;
    #20;
    $finish;



end




endmodule
