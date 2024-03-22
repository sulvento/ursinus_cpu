`include "isreg.v"

module testbench_isreg;

reg clk = 0;
reg loadsig;
wire [19:0] data_out;


isreg IS(
    .clk(clk),
    .loadsig(loadsig),
    .data_out(data_out)
);

always #5 clk = ~clk;

initial begin
    $dumpfile("testbench_isreg.vcd");
    $dumpvars(1, IS);
    
    loadsig = 0;
    #5;
    loadsig = 1;
    #5;
    $finish;




end


endmodule
