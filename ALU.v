/*

Implementation of an ALU 
Owen Fazzini
CS-274

*/

//This ALU is INCOMPLETE: it is a self-contained unit, with no integration with the wider CPU.


//Switch statement for every state (fetch, decode, execute, writeback, etc.)

//TODO: implement and test instruction count register

module register_file ( //code for dynamic GP registers, by jaiden
    input clk, reset, RegWrite,
    input [4:0] Rs1, Rs2, Rd, 
    input [15:0] Write_data,
    output [15:0] Read_data1, Read_data2
    );

reg [5:0] registers [19:0]; 
integer k;

assign Read_data1 = registers[Rs1];
assign Read_data2 = registers[Rs2];

always @ (posedge clk) begin
    if (reset == 1'b1) begin
        for (k = 0; k < 20; k = k + 1) begin
            registers[k] = 19'h0;
        end
    end
    else if (RegWrite == 1'b1) registers[Rd] = Write_data;
end

endmodule

module PointerRegs( //code for 6 pointer registers
    input clk,
    input [19:0] data_in,
    input [19:0] data_out
); 

//register    = value it points to
reg [19:0] IS = 20'h000; //instruction segment register
reg [19:0] DS = 20'h3DC; //dynamic segment register

//rest of registers will be implemented later
endmodule

module Memory(              //memory
    input addr,             //address
    input clk,              //clock
    inout [19:0] data,      //data -- either read or write
    input readsig,          //read signal
    input control,          //control signal
    input writesig          //write signal
); 

    reg [19:0] mem [0:64];  //64 words of 20 bits each
    reg [19:0] data_out;    //data output
    assign data = (control && readsig) ? data_out : 20'bz;

    initial begin           //initializes memory locations
        //HAVE TO REDO THIS: THIS IS ROM NOW. ROM AND RAM ARE SEPARATE MODULES
        //instruction set  
        mem[0] = 20'h000;   //TRAP 
        mem[1] = 20'h015;   //NOP  
        mem[2] = 20'h029;   //JMP  
        mem[3] = 20'h03E;   //JMPZ 
        mem[4] = 20'h053;   //JMPS 
        mem[5] = 20'h068;   //JMPZS 
        mem[6] = 20'h07D;   //LSTAT 
        mem[7] = 20'h092;   //XSTAT 
        mem[8] = 20'h0A7;   //NOT
        mem[9] = 20'h0BC;   //AND
        mem[10] = 20'h0D1;  //OR
        mem[11] = 20'h0E6;  //XOR
        mem[12] = 20'h0FB;  //SHFTR
        mem[13] = 20'h110;  //SHFTL
        mem[14] = 20'h125;  //ROTR
        mem[15] = 20'h13A;  //ROTL
        mem[16] = 20'h14F;  //SWAP
        mem[17] = 20'h164;  //INC
        mem[18] = 20'h179;  //DEC
        mem[19] = 20'h18E;  //ADD
        mem[20] = 20'h1A3;  //ADDC
        mem[21] = 20'h1B8;  //SUB
        mem[22] = 20'h1CD;  //SUBC
        mem[23] = 20'h1E2;  //EQ
        mem[24] = 20'h1F7;  //GT
        mem[25] = 20'h20C;  //LT
        mem[26] = 20'h221;  //GET
        mem[27] = 20'h236;  //LET
        mem[28] = 20'h24B;  //MRR
        mem[29] = 20'h260;  //LDC
        mem[30] = 20'h275;  //LDD
        mem[31] = 20'h28A;  //LDI
        mem[32] = 20'h29F;  //STD
        mem[33] = 20'h2B4;  //STI
        mem[34] = 20'h2C9;  //MUL -- extra credit
        mem[35] = 20'h2DE;  //DIV -- extra credit

        //registers
        mem[36] = 20'h2E0;  //SX -- status register
        mem[37] = 20'h30A;  //IC -- instruction count
        mem[38] = 20'h334;  //MA -- mem access count
        mem[39] = 20'h35E;  //MC -- mem correction count
        mem[40] = 20'h388;  //IS -- instruction segment
        mem[41] = 20'h3B2;  //DS -- dynamic segment

    end



    always @(posedge clk)
        if(control && writesig && ~readsig)
            mem[addr] = data;
        
    always @(posedge clk)
        if(control && readsig && ~writesig)
            data_out = mem[addr];
    
endmodule

module ICRegister( //instruction count register
    input [19:0] inst_comp,       //number of completed instructions
    input clk,
    input rst,
    output reg [19:0] inst_count  //return value
);

always @(posedge clk or posedge rst) begin //increments return value, or resets
    if (rst) begin
        inst_count <= 20'h0;
    end else begin
        inst_count <= inst_count + inst_comp;
    end
end

endmodule

module MARegister( //memory access register
    input [19:0] data_in,
    input clk,
    input [19:0] data_out
); 
    //TODO
endmodule

module MCRegister( //memory correction register
    input [19:0] data_in,
    input clk,
    input [19:0] data_out
); 

endmodule

module ControlUnit(); //control unit

endmodule

module StatusRegister( //status register

    input[12:0] ALU_in, //input from the ALU, telling what flags to set

    input clk, //clock signal
    input rst, //reset signal

    output reg ZE, //zero
    output reg N,  //negative
    output reg O,  //overflow
    output reg U,  //underflow
    output reg F,  //carry flag, full-word
    output reg L,  //carry flag, half-word low
    output reg I,  //carry flag, half-word high
    output reg D,  //division by zero flag
    output reg H,  //half-word mode flag 
    output reg S,  //same register flag
    output reg V,  //memory violation flag 
    output reg C,  //ECC circuit trigger flag 
    output reg T   //trap mode flag

);

//sets flags to corresponding states from ALU input

always @(posedge clk or posedge rst) begin
    if(rst) begin //resets all flags to 0
        ZE = 0;
        N = 0;
        O = 0;
        U = 0;
        F = 0;
        L = 0;
        I = 0;
        D = 0;
        H = 0;
        S = 0;
        V = 0;
        C = 0;
        T = 0;
    end else begin
        ZE = ALU_in[0];
        N = ALU_in[1];
        O = ALU_in[2];
        U = ALU_in[3];
        F = ALU_in[4];
        L = ALU_in[5];
        I = ALU_in[6];
        D = ALU_in[7];
        H = ALU_in[8];
        S = ALU_in[9];
        V = ALU_in[10];
        C = ALU_in[11];
        T = ALU_in[12];
end
end


endmodule

module ALU(

input [13:0] instruction, //name of the instruction called, control signal

input wire [19:0] A, B, //registers A, B 

input cin, clk, //carry in, clock

output reg [19:0] result, //output reg

output reg [12:0] SX_flags, //used to update SX

output reg [19:0] IC_count, //used to update IC

output reg carry_out //for adding/subtracting with carry
);


//connects status register
StatusRegister SX(
    .ALU_in(SX_flags),
    .clk(clk),
    .rst(rst),
    .ZE(ZE),
    .N(N),
    .O(O),
    .U(U),
    .F(F),
    .L(L),
    .I(I),
    .D(D),
    .H(H),
    .S(S),
    .V(V),
    .C(C),
    .T(T)
);

//connects instruction count register
ICRegister IC(
    .inst_comp(IC_count),
    .clk(clk),
    .rst(rst),
    .inst_count(inst_count)
);

always @(posedge clk) begin

    case(instruction) 

    //program flow
    13'h000: begin //TRAP
        SX_flags[12] = 1;
        //THIS INSTRUCTION IS INCOMPLETE, for now only sets flag.
        //will be fully implemented when rest of CPU is done.
    end
    13'h015: begin //NOP
        result = 0;
    end
    13'h029: begin //JMP
        //need program pointer to be integrated
    end
    13'h03E: begin //JMPZ
        if(SX_flags[0] == 1) begin
            //need program pointer for this
        end

    end
    13'h053: begin //JMPS
        if(SX_flags[1] == 1) begin
            //need program pointer for this
        end

    end    
    13'h068: begin //JMPZS
        if(SX_flags[1] == 1 & SX_flags[0] == 1) begin
            //need program pointer for this
        end

    end
    13'h07D: begin //LSTAT
        result = SX_flags;
    end
    13'h092: begin //XSTAT
        if(SX_flags[12] == 1) begin
            result = SX_flags ^ A;
        end
    end

    //logic cases
    13'h0A7: begin //NOT
        result = ~A;
    end
    13'h0D1: begin //OR
        result = A | B;
    end
    13'h0BC: begin //AND
        result = A & B;
    end
    13'h0E6: begin //XOR
        result = A ^ B;
    end

    //bitshifts
    13'h0FB: begin //SHFTR
        result = A >> B;
    end
    13'h110: begin //SHFTL
        result = A << B;
    end
    13'h125: begin //ROTR
        result = (A >> B) | (A << (20 - B));
    end
    13'h13A: begin //ROTL
        result = (A << B) | (A >> (20 - B));
    end

    /*13'h14F: begin //SWAP
        //SWAP instruction has to write one value into memory,
        //retrieve it, then set the other register equal to that value,
        //then vice versa. Memory isn't implemented yet; this has to be written later!
    end*/

    //arithmetic
    13'h164: begin //INC
        result = A + 1;
    end
    13'h179: begin //DEC
        result = A - 1;
    end
    13'h18E: begin //ADD, no carry
        result = A + B;
    end
    13'h1A3: begin //ADDC 
    //this is correct, but have to make it a full 20 bits
        result = A ^ B ^ cin;
        carry_out = (A & B) | (B & cin) | (cin & A);
    end
    13'h1B8: begin //SUB
        result = A - B;
    end
    13'h1CD: begin //SUBC
    //functional, but again have to make it full 20 bits
        result = A - B - cin;
        carry_out = (~A & B) | ((~A | B) & cin);
    end
    13'h2C9: begin //MUL: only do if i have time

    end
    13'h2DE: begin //DIV: again only do if i have time (MUL & div are extra credit)
    
    end

    //equality
    13'h1E2: begin //EQ
        if (A == B) begin
            SX_flags[0] = 1;
        end
        else begin
            SX_flags[0] = 0;
        end
    end
    13'h1F7: begin //GT
        if (A > B) begin
            SX_flags[1] = 1;
        end
        else begin
            SX_flags[1] = 0;
        end
    end
    13'h20C: begin //LT
        if (A < B) begin
            SX_flags[1] = 1;
        end
        else begin
            SX_flags[1] = 0;
        end
    end
    13'h221: begin //GET
        if (A >= B) begin
            SX_flags[0] = 1;
            SX_flags[1] = 0;
        end
        else begin
            SX_flags[0] = 0;
            SX_flags[1] = 1;
        end
    end
    13'h236: begin //LET
        if (A <= B) begin
            SX_flags[0] = 1;
            SX_flags[1] = 1;
        end
        else begin
            SX_flags[0] = 0;
            SX_flags[1] = 0;
        end
    end

    //memory instructions
    13'h24B: begin //MRR

    end
    13'h260: begin //LDC
    
    end
    13'h275: begin //LDD

    end
    13'h28A: begin //LDI

    end
    13'h29F: begin //STD

    end
    13'h2B4: begin //STI

    end

    default: result <= 0;
    endcase

end



endmodule

//ECC MEMORY CODE BELOW: have to integrate!

//synchronous d flip flop 
module dff(clk,reset,d,q);
input clk,reset,d;
output reg q;

always @ (posedge clk)begin
    if(reset)
        q <= 0;
    else
        q <= d;
end
                
endmodule

module ecc_mem(
  //clock signal and reset inputs, d + q inputs
  input clk, reset, d0, d1, d2,
  output wire q0, q1, q2,
  //q-hat output
  output reg [2:0] qh,
  //error flag output
  output reg er
);

//3 instantiations of circuits
dff dff0(
  .clk(clk),
  .reset(reset),
  .d(d0),
  .q(q0)
); 

dff dff1(
  .clk(clk),
  .reset(reset),
  .d(d1),
  .q(q1)
);

dff dff2(
  .clk(clk),
  .reset(reset),
  .d(d2),
  .q(q2)
);

//ECC logic
always @* begin
    qh[0] = (q0 & q1) | (q1 & q2) | (q0 & q2);
    qh[1] = (q0 & q1) | (q1 & q2) | (q0 & q2);
    qh[2] = (q0 & q1) | (q1 & q2) | (q0 & q2);
end

always @* begin
    er = (qh[0] ^ qh[1]) | (qh[1] ^ qh[2]) | (qh[2] ^ qh[0]);
end

endmodule
