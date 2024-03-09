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
    {NOT}: begin

    end
    {OR}: begin

    end
    {AND}: begin

    end
    {XOR}: begin

    end

    //Finish switch statement, implement logic


endcase



endmodule