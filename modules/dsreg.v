module dsreg (   //instruction segment register 
    input clk,                          //clock signal
    input loadsig,
    output reg [19:0] data_out 
);

always @(posedge clk) begin
    if(loadsig) begin
        data_out <= 20'h3DC;            //first address of RAM
    end
end

endmodule
