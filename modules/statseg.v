module statseg (
    input clk, reset,           //clock and reset signals
    input loadsig,              //load signal
    input [19:0] data_in,       //input
    output reg [19:0] data_out  //output
);

//whenever a constant is declared in a program
//and written to memory, this register will hold its address;
//input and output both represent that memory address

//the address held here, with an offset, will be used
//by other programs to find and use numeric constants

always @(posedge clk or posedge reset) begin
    if (reset) begin
        data_out <= 20'h0;
    end else if (loadsig) begin
        data_out <= data_in;
    end
end

endmodule
