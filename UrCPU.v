module UrCPU (
input clk, reset
);


wire clk, reset, RegWrite;
wire [4:0] Rs1, Rs2, Rd;
wire [15:0] Write_data, Read_data1, Read_data2;
register_file register_file (
    .clk(clk), 
    .reset(reset),
    .RegWrite(RegWrite),
    .Rs1(Rs1),
    .Rs2(Rs2),
    .Rd(Rd),
    .Write_data(Write_data),
    .Read_data1(Read_data1),
    .Read_data2(Read_data2)
); 


wire load_dp, inc_dp, write_dp;
wire [15:0] dp_data_in, dp_data_out;
dynamic_pointer dynamic_pointer (
    .clk(clk),
    .reset(reset),
    .load_dp(load_dp),
    .inc_dp(inc_dp),
    .write_dp(write_dp),
    .dp_data_in(dp_data_in),
    .dp_data_out(dp_data_out)
  );

wire clk, reset; 
wire load_dsr;
wire [15:0] dsr_data_in, dsr_data_out;
dynamic_segment_register dynamic_segment_register (
    .clk(clk),
    .reset(reset),
    .load_dsr(load_dsr),
    .dsr_data_in(dsr_data_in),
    .dsr_data_out(dsr_data_out)
  );

instruction_count_register dut (
    .clk(clk),
    .reset(reset),
    .update_count(update_count),
    .count_out(count_out)
  );

wire load_ip, inc_ip;
wire [15:0] ip_data_in, ip_data_out;
instruction_pointer instruction_pointer (
    .clk(clk),
    .reset(reset),
    .load_ip(load_ip),
    .inc_ip(inc_ip),
    .ip_data_in(ip_data_in),
    .ip_data_out(ip_data_out)
  );

wire load_isr;
wire [15:0] isr_data_in, isr_data_out;
instruction_segment_register instruction_segment_register (
    .clk(clk),
    .reset(reset),
    .load_isr(load_isr),
    .isr_data_in(isr_data_in),
    .isr_data_out(isr_data_out)
  );

memory_access_register memory_access_register (
    .clk(clk),
    .reset(reset),
    .count_access(count_access),
    .access_count_out(access_count_out)
  );

memory_correction_register memory_correction_register (
    .clk(clk),
    .reset(reset),
    .error_correction_event(error_correction_event),
    .correction_count_out(correction_count_out)
  );

wire load_sp, inc_sp, write_sp;
wire [15:0] sp_data_in, sp_data_out, invalid_write;
static_pointer static_pointer (
    .clk(clk),
    .reset(reset),
    .load_sp(load_sp),
    .inc_sp(inc_sp),
    .write_sp(write_sp),
    .sp_data_in(sp_data_in),
    .sp_data_out(sp_data_out),
    .invalid_write(invalid_write)
  );

wire load_ssr;
wire [15:0] ssr_data_in, ssr_data_out;
static_segment_register static_segment_register (
    .clk(clk),
    .reset(reset),
    .load_ssr(load_ssr),
    .ssr_data_in(ssr_data_in),
    .ssr_data_out(ssr_data_out)
  );

endmodule