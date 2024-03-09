/*

Implementation of an ALU 
Owen Fazzini
CS-274

//TODO: Fill in outlines. Registers should be made first probably. See about implementing switch statement.
//Fill out full switch statement with hex values.

*/


module ALU(
input [15:0] instruction, //name of the instruction called
input A,
input B,
output result
);

case(instruction) 

    //logic cases
    {16'h0x1000}: begin //NOT
        result = ~a;
    end
    {16'h0x1002}: begin //OR
        result = a | b;
    end
    {16'h0x1001}: begin //AND
        result = a & b;
    end
    {16'h0x1003}: begin //XOR
        result = a ^ b;
    end

    //bitshifts
    {16'h0x0010}: begin //shift right
        result = a >> b;
    end
    {16'h0x0011}: begin //shift left
        result = a << b;
    end
    {16'h0x0012}: begin //rotate right

    end
    {16'h0x0013}: begin //rotate left

    end
    {16'h0x0014}: begin //swap

    end

    




endcase



endmodule
