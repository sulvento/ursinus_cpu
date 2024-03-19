/*

Implementation of an ALU 
Owen Fazzini
CS-274

*/

//This ALU is INCOMPLETE: it is a self-contained unit, with no integration with the wider CPU.

//Switch statement for every state (fetch, decode, execute, writeback, etc.)


module ALU(
input [13:0] instruction, //name of the instruction called, control signal
input [19:0] A, //register A 
input [19:0] B, //register B
output reg [19:0] result, //output reg
output reg carry_out //for adding/subtracting with carry
);

integer carry_in;
integer borrow_in;

always@* begin

    case(instruction) 

    //logic cases
    {13'h0A7}: begin //NOT
        result = ~A;
    end
    {13'h0D1}: begin //OR
        result = A | B;
    end
    {13'h0BC}: begin //AND
        result = A & B;
    end
    {13'h0E6}: begin //XOR
        result = A ^ B;
    end

    //bitshifts
    {13'h0FB}: begin //SHFTR
        result = A >> B;
    end
    {13'h110}: begin //SHFTL
        result = A << B;
    end
    {13'h125}: begin //ROTR
        result = (A >> B) | (A << (20 - B));
    end
    {13'h13A}: begin //ROTL
        result = (A << B) | (A >> (20 - B));
    end
    /*{13'h14F}: begin //SWAP
        A = B;
    end*/

    //arithmetic
    /*{13'h164}: begin //INC
        A = A + 1;
    end*/
    /*{13'h179}: begin //DEC
        A = A - 1;
    end*/
    {13'h18E}: begin //ADD, no carry
        result = A + B;
    end
    {13'h1A3}: begin //ADDC 
        result = A + B + carry_in;
        carry_out = (result < A) || (result < B);
    end
    {13'h1B8}: begin //SUB
        result = A - B;
    end
    {13'h1CD}: begin //SUBC
        result = A - B - borrow_in;
        carry_out = (A < B) || ((A==B) && !borrow_in);
    end

    //equality: commented out for now until status register is finished/fully implemented
    /*{13'h1E2}: begin //EQ
        if (A == B) begin
            Z = 1;
        end
        else begin
            Z = 0;
        end
    end*/
    /*{13'h1F7}: begin //GT
        if (A > B) begin
            N = 1;
        end
        else begin
            N = 0;
        end
    end*/
    /*{13'h20C}: begin //LT
        if (A < B) begin
            N = 1;
        end
        else begin
            N = 0;
        end
    end*/
    /*{13'h221}: begin //GET
        if (A >= B) begin
            Z = 1;
            N = 0;
        end
        else begin
            Z = 0;
            N = 1;
        end
    end*/
    /*{13'h236}: begin //LET
        if (A <= B) begin
            Z = 1;
            N = 1;
        end
        else begin
            Z = 0;
            N = 0;
        end*/
    endcase

end



endmodule
