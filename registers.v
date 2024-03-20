module register_file (clk, reset, Rs1, Rs2, Rd, Write_data, RegWrite, Read_data1, Read_data2);
input clk, reset, RegWrite;
input [4:0] Rs1, Rs2, Rd;
input [15:0] Write_data;

output [15:0] Read_data1, Read_data2;

reg [15:0] registers [5:0]; 
integer k;

assign Read_data1 = registers[Rs1];
assign Read_data2 = registers[Rs2];

always @ (posedge clk)
begin
if (reset == 1'b1)
begin
for (k = 0; k < 16; k = k + 1)
begin
    registers[k] = 16'h0;
end
end
else if (RegWrite == 1'b1) registers[Rd] = Write_data;
end

endmodule
