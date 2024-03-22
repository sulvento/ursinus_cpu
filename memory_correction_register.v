module memory_correction_register (
    input clk,
    input reset,
    input error_correction_event,  
    output reg [15:0] correction_count_out 
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        correction_count_out <= 16'd0;
    end else if (error_correction_event) begin
        correction_count_out <= correction_count_out + 16'd1;
    end
end

endmodule
