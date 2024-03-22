module isreg (   //instruction segment register 
    input clk,                          //clock signal
    input loadsig,
    output reg [19:0] data_out 
);

always @(posedge clk) begin
    if(loadsig) begin
        data_out <= 20'h000;            //ROM address of TRAP, the first instruction stored in memory
    end
end

endmodule
