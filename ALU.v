/*

Implementation of an ALU 
Owen Fazzini
CS-274

*/

//This ALU is INCOMPLETE: it is a self-contained unit, with no integration with the wider CPU.

//Switch statement for every state (fetch, decode, execute, writeback, etc.)


module ALU(
input [19:0] instruction, //name of the instruction called, control signal
input [19:0] A, //register A 
input [19:0] B, //register B
output [19:0] result //output reg
output carry_out; //for adding/subtracting with carry
);

case(instruction) 

    //logic cases
    {20'h0x000A1}: begin //NOT
        result = ~a;
    end
    {20'h0x000C9}: begin //OR
        result = a | b;
    end
    {20'h0x000B5}: begin //AND
        result = a & b;
    end
    {20'h0x000DD}: begin //XOR
        result = a ^ b;
    end

    //bitshifts
    {20'h0x000F1}: begin //SHFTR
        result = a >> b;
    end
    {20'h0x00105}: begin //SHFTL
        result = a << b;
    end
    {20'h0x00119}: begin //ROTR
        result = (a >> b) | (a << (20 - b));
    end
    {20'h0x0012D}: begin //ROTL
        result = (a << b) | (a >> (20 - b));
    end
    {20'h0x00011}: begin //SWAP
        a = b;
    end

    //arithmetic
    {20'h0x00141}: begin //INC
        a = a + 1;
    end
    {20'h0x00155}: begin //DEC
        a = a - 1;
    end
    {20'h0x00169}: begin //ADD, no carry
        result = a + b;
    end
    {20'h0x0017D}: begin //ADDC 
        integer carry_in;
        result = a + b + carry_in;
        carry_out = (result < a) || (result < b);
    end
    {20'h0x00191}: begin //SUB
        result = a - b;
    end
    {20'h0x001A5}: begin //SUBC
        integer borrow_in;
        result = a - b - borrow_in;
        carry_out = (a < b) || ((a==b) && !borrow_in);
    end

    //equality
    {20'h0x001B9}: begin //EQ
        if a = b begin
            Z = 1;
        end
        else begin
            Z = 0;
        end
    end
    {20'h0x001CD}: begin //GT
        if a > b begin
            N = 1;
        end
        else begin
            N = 0;
        end
    end
    {20'h0x001E1}: begin //LT
        if a < b begin
            N = 1;
        end
        else begin
            N = 0;
        end
    end
    {20'h0x001F5}: begin //GET
        if a >= b begin
            Z = 1;
            N = 0;
        end
        else begin
            Z = 0;
            N = 1;
        end
    end
    {20'h0x00209}: begin //LET
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
