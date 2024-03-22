module dynamic_pointer (
    input clk,
    input reset,
    input load_dp,
    input inc_dp,  
    input write_dp, 
    input [15:0] dp_data_in,  
    output reg [15:0] dp_data_out 
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        dp_data_out <= 16'd0; 
    end else begin 
        if (load_dp) begin
            dp_data_out <= dp_data_in;
        end else if (inc_dp) begin
            dp_data_out <= dp_data_out + 16'd1; 
        end else if (write_dp) begin
            dp_data_out <= dp_data_in;
        end
    end
end

endmodule
