`include "dsreg.v"

module testbench_dsreg;

reg clk = 0;
reg loadsig;
wire [19:0] data_out;


dsreg DS(
    .clk(clk),
    .loadsig(loadsig),
    .data_out(data_out)
);

always #5 clk = ~clk;

initial begin
    $dumpfile("testbench_dsreg.vcd");
    $dumpvars(1, DS);
    
    loadsig = 0;
    #5;
    loadsig = 1;
    #5;
    $finish;




end


endmodule
