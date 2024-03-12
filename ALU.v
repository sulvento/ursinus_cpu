/*

Implementation of an ALU 
Owen Fazzini
CS-274

*/

//This ALU is INCOMPLETE: it is a self-contained unit, with no integration with the wider CPU.


module ALU(
input [19:0] instruction, //name of the instruction called
input [19:0] A, //register A 
input [19:0] B, //register B
output [19:0] result //output reg
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
    {20'h0x0012}: begin //ROTR

    end
    {20'h0x0013}: begin //ROTL

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

    end


    




endcase



endmodule
