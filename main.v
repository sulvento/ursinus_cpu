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

//instructions: ALU. 

//program flow
enum TRAP = 16'h0x0001; //trap mode
enum NOP = 16'h0x0002; //no operation
enum JMP = 16'h0x0003; //jump unconditional
enum JMPZ = 16'h0x0004; //jump zero
enum JMPS = 16'h0x0005; //jump sign
enum JMPZS = 16'h0x0006; //jump zero-sign
enum LSTAT = 16'h0x0007; //load status register
enum XSTAT = 16'h0x0008;  //xor status register

//logic
enum NOT = 16'h0x1000;
enum AND = 16'h0x1001;
enum OR = 16'h0x1002; 
enum XOR = 16'h0x1003;

//bit shifts
enum SHFTR = 16'h0x0010; //shift right
enum SHFTL = 16'h0x0011; //shift left
enum ROTR = 16'h0x0012; //rotate right
enum ROTL = 16'h0x0013; //rotate left
enum SWAP = 16'h0x0014; //swap

//arithmetic
enum INC = 16'h0x0100; //increment
enum DEC = 16'h0x0101; //decrement
enum ADD = 16'h0x0102; //add, no carry
enum ADDC = 16'h0x0103; //add, carry
enum SUB = 16'h0x0104; //subtract no carry
enum SUBC = 16'h0x0105; //subtract, carry

//comparisons
enum EQ = 16'h0x0200; //equal to
enum GT = 16'h0x0201; //greater than
enum LT = 16'h0x0202; //less than
enum GET = 16'h0x0203; //greater or equal to
enum LET = 16'h0x0204; //less or equal to

//instructions: memory

enum MRR = 16'h0x0300; //load memory register to register
enum LDC = 16'h0x0301; //load num constant to register
enum LDD = 16'h0x0302; //load direct data to register
enum LDI = 16'h0x0303; //load indirect data to register
enum STD = 16'h0x0304; //store direct
enum STI = 16'h0x0305; //store indirect



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
