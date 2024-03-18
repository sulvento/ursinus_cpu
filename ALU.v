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
output [19:0] result //output reg
output carry_out; //for adding/subtracting with carry
);

case(instruction) 

    //logic cases
    {13'h0x0A7}: begin //NOT
        result = ~a;
    end
    {13'h0x0D1}: begin //OR
        result = a | b;
    end
    {13'h0x0BC}: begin //AND
        result = a & b;
    end
    {13'h0x0E6}: begin //XOR
        result = a ^ b;
    end

    //bitshifts
    {13'h0x0FB}: begin //SHFTR
        result = a >> b;
    end
    {13'h0x110}: begin //SHFTL
        result = a << b;
    end
    {13'h0x125}: begin //ROTR
        result = (a >> b) | (a << (20 - b));
    end
    {13'h0x13A}: begin //ROTL
        result = (a << b) | (a >> (20 - b));
    end
    {13'h0x14F}: begin //SWAP
        a = b;
    end

    //arithmetic
    {13'h0x164}: begin //INC
        a = a + 1;
    end
    {13'h0x179}: begin //DEC
        a = a - 1;
    end
    {13'h0x18E}: begin //ADD, no carry
        result = a + b;
    end
    {13'h0x1A3}: begin //ADDC 
        integer carry_in;
        result = a + b + carry_in;
        carry_out = (result < a) || (result < b);
    end
    {13'h0x1B8}: begin //SUB
        result = a - b;
    end
    {13'h0x1CD}: begin //SUBC
        integer borrow_in;
        result = a - b - borrow_in;
        carry_out = (a < b) || ((a==b) && !borrow_in);
    end

    //equality
    {13'h0x1E2}: begin //EQ
        if a = b begin
            Z = 1;
        end
        else begin
            Z = 0;
        end
    end
    {13'h0x1F7}: begin //GT
        if a > b begin
            N = 1;
        end
        else begin
            N = 0;
        end
    end
    {13'h0x20C}: begin //LT
        if a < b begin
            N = 1;
        end
        else begin
            N = 0;
        end
    end
    {13'h0x221}: begin //GET
        if a >= b begin
            Z = 1;
            N = 0;
        end
        else begin
            Z = 0;
            N = 1;
        end
    end
    {13'h0x236}: begin //LET
        if a <= b begin
            Z = 1;
            N = 1;
        end
        else begin
            Z = 0;
            N = 0;
        end
    end

endcase



endmodule
