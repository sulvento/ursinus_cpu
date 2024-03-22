module dynamic_segment_register (
    input clk,
    input reset,
    input load_dsr,
    input [15:0] dsr_data_in,  
    output reg [15:0] dsr_data_out 
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        dsr_data_out <= 16'd0; 
    end else if (load_dsr) begin
        dsr_data_out <= dsr_data_in;
    end
end

endmodule
