module memory_access_register (
    input clk,
    input reset,
    input count_access,  
    output reg [15:0] access_count_out 
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        access_count_out <= 16'd0;
    end else if (count_access) begin
        access_count_out <= access_count_out + 16'd1;
    end
end

endmodule
