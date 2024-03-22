module static_segment_register (
    input clk,
    input reset,
    input load_ssr,
    input [15:0] ssr_data_in,  
    output reg [15:0] ssr_data_out 
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        ssr_data_out <= 16'd0; 
    end else if (load_ssr) begin
        ssr_data_out <= ssr_data_in;
    end
end

endmodule
