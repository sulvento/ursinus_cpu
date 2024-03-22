module static_pointer (
    input clk,
    input reset,
    input load_sp,
    input inc_sp,  
    input write_sp, 
    input [15:0] sp_data_in,  
    output reg [15:0] sp_data_out,
    output reg invalid_write  
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        sp_data_out <= 16'd0; 
        invalid_write <= 1'b0; 
    end else begin 
        if (load_sp) begin
            sp_data_out <= sp_data_in;
        end else if (inc_sp) begin
            sp_data_out <= sp_data_out + 16'd1; 
        end

        if (write_sp) begin
            invalid_write <= 1'b1; 
        end else begin
            invalid_write <= 1'b0;  
        end
    end
end

endmodule
