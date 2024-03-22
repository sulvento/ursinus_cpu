module dynamic_pointer (            //dynamic pointer
    input clk,
    input reset,
    input inc, 
    input writesig,
    input readsig,
    input [19:0] data_in, 
    output reg [19:0] data_out
);

reg [19:0] DP;                      //declares the actual register

integer k;

always @(posedge clk) begin         //initializes DP to 0 if reset is true
    if (reset) begin
        for (k = 0; k < 20; k = k + 1) begin
                DP[k] <= 0;
            end
    end 
    
end  

always @(posedge clk) begin
    if(readsig)
        data_out <= DP;
    else if(writesig)
        DP <= data_in;
    else if(inc)
        DP <= DP + 1;
end


endmodule
