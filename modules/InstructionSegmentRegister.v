module instruction_segment_register (
    input clk,
    input reset,
    input load_isr,
    input [15:0] isr_data_in,  
    output reg [15:0] isr_data_out 
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        isr_data_out <= 16'd0; 
    end else if (load_isr) begin
        isr_data_out <= isr_data_in;
    end
end

endmodule
