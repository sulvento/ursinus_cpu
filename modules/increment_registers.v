//INCREMENT REGISTERS

module ICRegister( //instruction count register
    input clk,
    input rst,
    input inc,
    output reg [19:0] inst_count  //return value
);

always @(posedge clk or posedge rst) begin //increments return value, or resets
    if (rst) begin
        inst_count <= 20'b0;
    end else (inc) begin
        inst_count <= inst_count + 1;
    end
end

endmodule

module MARegister( //memory access register
    input clk,
    input rst,
    input inc,
    output reg [19:0] ma_count  //return value
);

always @(posedge clk or posedge rst) begin //increments return value, or resets
    if (rst) begin
        ma_count <= 20'b0;
    end else (inc) begin
        ma_count <= ma_count + 1;
    end
end

endmodule

module MCRegister( //memory correction register
    input clk,
    input rst,
    input inc,
    output reg [19:0] mc_count  //return value
);

always @(posedge clk or posedge rst) begin //increments return value, or resets
    if (rst) begin
        mc_count <= 20'b0;
    end else (inc) begin
        mc_count <= mc_count + 1;
    end
end

endmodule
