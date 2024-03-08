//MAIN CPU
//MEMORY AND CPU
 
module StatusRegister (
    input wire clk, reset,
    input wire [11:0] data,
    output reg Z, N, O, U, FW, HWL, HWH, DZ,
    HWM, SR, MV, MC, TRAP);

    typedef struct{
        logic Z; //zero flag
        logic N; //negative flag
        logic O; //overflow flag
        logic U; //underflow flag
        logic FW; //overflow w carry
        logic HWL; //half-word overflow low
        logic HWH; //half-word overflow high
        logic DZ; //division by zero flag
        logic HWM; //half-word mode flag
        logic SR; //same register flag
        logic MV; //memory violation
        logic MC; //memory corruption
        logic TRAP; //trap mode
    } flagslist;

    flagslist flags;

    always_ff @(posedge clk or negedge reset) begin
        if(~reset) begin
            flags <= '0; //resets flags
        end else begin
            flags.Z <= data[0];
            flags.N <= data[1];
            flags.O <= data[2];
            flags.U <= data[3];
            flags.FW <= data[4];
            flags.HWL <= data[5];
            flags.HWH <= data[6];
            flags.DZ <= data[7];
            flags.HWM <= data[8];
            flags.SR <= data[9];
            flags.MV <= data[10];
            flags.MC <= data[11];
            flags.TRAP <= data[12];
    end
    end
    assign {Z, N, O, U, FW, HWL, HWH, DZ, HWM, SR, MV, MC, TRAP} = {flags.Z, flags.N, flags.O, flags.U, flags.FW, flags.HWL, flags.HWH,
    flags.DZ, flags.HWM, flags.SR, flags.MV, flags.MC, flags.TRAP};
endmodule

module CPU(
    input wire clk,
    input wire rst_n
);

typedef struct{
    integer Data[MAX_MEM];  
} MEM;

enum logic [19:0] MAX_MEM = 2**20;

//instructions: ALU. TODO: add in memory addresses

//program flow
enum TRAP; //trap mode
enum NOP; //no operation
enum JMP; //jump unconditional
enum JMPZ; //jump zero
enum JMPS; //jump sign
enum JMPZS; //jump zero-sign
enum LSTAT; //load status register
enum XSTAT;  //xor status register

//logic
enum NOT;
enum AND;
enum OR; 
enum XOR;

//bit shifts
enum SHFTR; //shift right
enum SHFTL; //shift left
enum ROTR; //rotate right
enum ROTL; //rotate left
enum SWAP; //swap

//arithmetic
enum INC; //increment
enum DEC; //decrement
enum ADD; //add, no carry
enum ADDC; //add, carry
enum SUB; //subtract no carry
enum SUBC; //subtract, carry

//comparisons
enum EQ; //equal to
enum GT; //greater than
enum LT; //less than
enum GET; //greater or equal to
enum LET; //less or equal to

//instructions: memory

enum MRR; //load memory register to register
enum LDC; //load num constant to register
enum LDD; //load direct data to register
enum LDI; //load indirect data to register
enum STD; //store direct
enum STI; //store indirect



reg [15:0] PC; //program counter
reg [15:0] SP; //stack pointer

reg [7:0] A; //accumulator
reg [7:0] X, Y; //index registers

wire C, Z, I, D, B, V, N; //status flags

StatusRegister sr_inst(
    .clk(clk), .rst_n(rst_n),
    .C(C), .Z(Z), .I(I), .D(D),
    .B(B), .V(V), .N(N)
);

task reset_routine;
    //resets PC, SP, and flags
    PC = 16'h0xFFFC;
    SP = 16'h0x0100;
    C, Z, I, D, B, V, N = 0'b0;
    A, X, Y = 0'b0;
endtask

MEM memory_inst;

task init_memory;
    //initializes memory to 0
    for(integer i = 0; i< MAX_MEM; i++)
    {
        Data[i] = 0'b0;
    }
endtask

task readmem(input integer address, output reg [7:0] value);
    value = memory_inst.Data[address];
endtask

reg [16:0] cycles = 0;

function fetch_instruction;
    //instruction fetch
    Data = MEM[PC];
    PC++;
    cycles--;
endfunction

task exec_instruction;
    //executes instruction
    while(cycles > 0) begin

    end
    

endtask

endmodule