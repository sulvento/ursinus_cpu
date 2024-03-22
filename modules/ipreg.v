module ipreg (                      //instruction pointer register
    input clk, reset,
    input inc,
    input [19:0] data_in,  
    output reg [19:0] data_out 
);

//this register contains the offset added to the instruction segment register
//to find instructions in memory.

always @(posedge clk or posedge reset) begin
    if (reset) begin
        data_out <= 20'b0; 
    end else if (inc) begin
        data_out <= data_out + 1;           //increments pointer by 1 
    end
end

endmodule
