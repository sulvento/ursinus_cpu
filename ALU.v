/*

Implementation of an ALU 
Owen Fazzini
CS-274

*/

//This ALU is INCOMPLETE: it is a self-contained unit, with no integration with the wider CPU.
//FUCKING SWAP INSTRUCTION AHHHH


//Switch statement for every state (fetch, decode, execute, writeback, etc.)
//TODO: Integrate flags! Status register


module ALU(
input [13:0] instruction, //name of the instruction called, control signal
input wire [19:0] A, //register A 
input wire [19:0] B, //register B
input cin, //carry in
input wire clk,
output reg [19:0] result, //output reg
output reg carry_out //for adding/subtracting with carry
);

integer iter;


always @(posedge clk) begin

    case(instruction) 

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

    //equality: commented out for now until status register is finished/fully implemented
    /*13'h1E2: begin //EQ
        if (A == B) begin
            Z = 1;
        end
        else begin
            Z = 0;
        end
    end*/
    /*13'h1F7: begin //GT
        if (A > B) begin
            N = 1;
        end
        else begin
            N = 0;
        end
    end*/
    /*13'h20C: begin //LT
        if (A < B) begin
            N = 1;
        end
        else begin
            N = 0;
        end
    end*/
    /*13'h221: begin //GET
        if (A >= B) begin
            Z = 1;
            N = 0;
        end
        else begin
            Z = 0;
            N = 1;
        end
    end*/
    /*13'h236: begin //LET
        if (A <= B) begin
            Z = 1;
            N = 1;
        end
        else begin
            Z = 0;
            N = 0;
        end*/
    default: result <= 0;
    endcase

end



endmodule
