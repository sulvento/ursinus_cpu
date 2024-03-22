module instruction_pointer (
    input clk,
    input reset,
    input load_ip,
    input inc_ip,  
    input [15:0] ip_data_in,  
    output reg [15:0] ip_data_out 
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        ip_data_out <= 16'd0; 
    end else if (load_ip) begin
        ip_data_out <= ip_data_in;
    end else if (inc_ip) begin
        ip_data_out <= ip_data_out + 16'd1; 
    end
end

endmodule
