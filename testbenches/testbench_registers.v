`include "registers.v"

module testbench_registers;

reg clk = 0;
reg reset, readsig, writesig;

reg halfword;

reg [2:0] sr, dr;
reg [19:0] data_write;
wire [19:0] data_read;

registers reggie(
    .clk(clk),
    .reset(reset),
    .readsig(readsig),
    .writesig(writesig),
    .sr(sr),
    .dr(dr),
    .halfword(halfword),
    .data_write(data_write),
    .data_read(data_read)
);

always #5 clk = ~clk;

initial begin

    $dumpfile("testbench_registers.vcd");
    $dumpvars(1,reggie);

    //simple full-word mode test; writes and reads data from reg AX
    halfword = 0;
    writesig = 1;
    readsig = 0;
    dr = 0;
    data_write = 20'b01010101;
    #10;

    writesig = 0;
    readsig = 1;
    sr = 0;
    #10;

    reset = 1;
    #10;
    reset = 0;

    //simple half-word mode test; writes and reads data from reg AXL
    halfword = 1;
    writesig = 1;
    readsig = 0;
    dr = 0;
    data_write = 10'b1;
    #10;

    writesig = 0;
    readsig = 1;
    sr = 0;
    #10;

    reset = 1;

    $finish;





end


endmodule
