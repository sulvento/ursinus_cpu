module instruction_count_register (
    input clk,
    input reset,
    input update_count,  
    output reg [15:0] count_out 
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        count_out <= 16'd0;
    end else if (update_count) begin
        count_out <= count_out + 16'd1;
    end
end

endmodule
