module spreg (                      //no testbench included, this register is functionally identical to IP          
    input clk,
    input reset,
    input inc,   
    input [19:0] data_in, 
    output reg [19:0] data_out
);

//static pointer contains offset added to static segment
//in order to locate static data. e.g.
//value A at address X;
//X is pointed to by static segment,
//this register contains the offset needed to find value B
//at address X + 1.

always @(posedge clk or posedge reset) begin
    if (reset) begin
        data_out <= 20'b0; 
    end else if (inc) begin
        data_out <= data_out + 1;           //increments pointer by 1 
    end
end

endmodule
