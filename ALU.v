/*

Implementation of an ALU 
Owen Fazzini
CS-274

*/

//This ALU is INCOMPLETE: it is a self-contained unit, with no integration with the wider CPU.


//Switch statement for every state (fetch, decode, execute, writeback, etc.)



module StatusRegister(

    input[12:0] ALU_in, //input from the ALU, telling what flags to set

    input clk, //clock signal

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

always @(posedge clk) begin
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



endmodule

module ALU(
input [13:0] instruction, //name of the instruction called, control signal

input wire [19:0] A, B, //registers A, B 

input cin, clk, //carry in, clock

output reg [19:0] result, //output reg

output reg [12:0] SX_flags, //used to update SX

output reg carry_out //for adding/subtracting with carry
);


//connects status register
StatusRegister SX(
    .ALU_in(SX_flags),
    .clk(clk),
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
