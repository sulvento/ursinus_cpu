module ALU(

input [13:0] instruction, //name of the instruction called, control signal

input wire [19:0] A, B, //registers A, B 

input cin, clk, rst, //carry in, clock, reset 

output reg [19:0] result, //output reg

output reg [19:0] SX_flags, //used to update SX

output reg reset,           //reset output signal

output reg carry_out, //for adding/subtracting with carry

output reg halfword,         //for half-word mode

output reg inc                  //register increment instructions



);

StatusRegister SX(              //initializes status register
    .ALU_in(SX_flags),
    .clk(clk),
    .rst(rst),
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

always @(posedge clk or posedge rst) begin      //initializes status register to 0
    if (rst) begin
        SX_flags <= 0;
    end
end

isreg IS (                                      //instruction segment register instantiation
    .clk(clk),                                  //clock signal
    .loadsig(loadsig),
    .is_data_out(is_data_out)
);

ipreg IP (                                      //instruction pointer register instantiation
    .clk(clk),
    .inc(inc),
    .reset(rst),
    .ip_data_in(result),
    .ip_data_out(ip_data_out)
);

registers regs(                                 //instantiates registers
    .clk(clk),
    .reset(reset),
    .reg_writesig(reg_writesig),
    .reg_readsig(reg_readsig),
    .halfword(halfword),
    .sr(sr),
    .dr(dr),
    .data_write(data_write),
    .data_read(data_read)
);

ROM romald(
    .rom_addr(rom_addr),
    .readsignal(readsignal),
    .control(control),
    .rom_data(rom_data)
);



always @(posedge clk) begin

    case(instruction) 

    //program flow
    13'h000: begin //TRAP
        SX_flags[12] = 1;       //sets status register position 12 to 1, triggering TRAP mode
        //TRAP MODE ACTIVATED
            //what does this do?
            //1. Sets a flag in SX. this flag is equivalent to an input in SX, which 
            //allows SX to be written to.
            //2. halts instruction of the control unit  [implementing later]

        //THIS INSTRUCTION IS INCOMPLETE, for now only sets flag.

    end
    13'h015: begin //NOP
        //nothing happens
    end
    13'h029: begin //JMP
        //uses instruction pointer (offset) and instruction segment (base) registers
        //to find location of next instruction to be fetched from memory.
        //accepts the contents of a general purpose register as input.
        if (rst) begin
            reset = 1;
        end
        else begin
            inc = 1;      
              //in GP reg, add pointer to segment reg to find location of instruction
        end
    end


    13'h03E: begin //JMPZ
        if(ZE == 1) begin
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

module StatusRegister( //status register

    input[19:0] ALU_in, //input from the ALU, telling what flags to set

    input clk, //clock signal
    input rst, //reset signal

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

always @(posedge clk or posedge rst) begin
    if(rst) begin //resets all flags to 0
        ZE = 0;
        N = 0;
        O = 0;
        U = 0;
        F = 0;
        L = 0;
        I = 0;
        D = 0;
        H = 0;
        S = 0;
        V = 0;
        C = 0;
        T = 0;
    end else begin
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
end


endmodule

module ipreg (                      //instruction pointer register
    input clk, reset,
    input inc,
    input [19:0] ip_data_in,  
    output reg [19:0] ip_data_out 
);

//this register contains the offset added to the instruction segment register
//to find instructions in memory.

always @(posedge clk or posedge reset) begin
    if (reset) begin
        ip_data_out <= 20'b0; 
    end else if (inc) begin
        ip_data_out <= ip_data_in;           //changes data
    end
end

endmodule

module isreg (   //instruction segment register 
    input clk,                          //clock signal
    input loadsig,
    output reg [19:0] is_data_out 
);

always @(posedge clk) begin
    if(loadsig) begin
        is_data_out <= 20'h000;            //ROM address of TRAP, the first instruction stored in memory
    end
end

endmodule

module registers (                  //code for dynamic GP registers, by jaiden
    input clk, reset,               //clock signal, reset signal
    input reg_writesig,                 //write signal
    input reg_readsig,                  //read signal

    input halfword,                 //half-word mode

    input [3:0] sr, dr,             //selection register, output register (for read/write, respectively)
    input [19:0] data_write,        //write input
    output reg [19:0] data_read     //read output
    );

    reg[19:0] AX, BX, CX, DX, EX, FX; //declares GP regs; full-word access

    reg[9:0] AXL, BXL, CXL, DXL, EXL, FXL; //declares half-word low regs
    reg[9:0] AXH, BXH, CXH, DXH, EXH, FXH; //declares half-word high regs

    integer k;                        //initializes regs to 0 if reset is active
    always @ (posedge clk) begin
        if (reset && halfword == 0) begin
            for (k = 0; k < 20; k = k + 1) begin
                AX[k] <= 0;
                BX[k] <= 0;
                CX[k] <= 0;
                DX[k] <= 0;
                EX[k] <= 0;
                FX[k] <= 0;
            end
        end
        else if(reset && halfword == 1) begin   //initializes low/high regs to 0 if reset active
            for(k =0; k < 20; k = k + 1) begin
                AXL[k] <= 0;
                BXL[k] <= 0;
                CXL[k] <= 0;
                DXL[k] <= 0;
                EXL[k] <= 0;
                FXL[k] <= 0;
                AXH[k] <= 0;
                BXH[k] <= 0;
                CXH[k] <= 0;
                DXH[k] <= 0;
                EXH[k] <= 0;
                FXH[k] <= 0;
            end
    end
    end

    always @(posedge clk) begin                 //switch statement for reading from regs
        if(reg_readsig && halfword == 0)
            case(sr)
                0: data_read = AX;
                1: data_read = BX;
                2: data_read = CX;
                3: data_read = DX;
                4: data_read = EX;
                5: data_read = FX;
                default: data_read = 20'b0;
            endcase
        else if(reg_readsig && halfword == 1)
            case(sr)
                //AX
                0: data_read = AXL;
                1: data_read = AXH;
                //BX
                2: data_read = BXL;
                3: data_read = BXH;
                //CX
                4: data_read = CXL;
                5: data_read = CXH;
                //DX
                6: data_read = DXL;
                7: data_read = DXH;
                //EX
                8: data_read = EXL;
                9: data_read = EXH;
                //FX
                10: data_read = FXL;
                11: data_read = FXH;
            endcase
        end

        always @(posedge clk) begin                 //switch statement for writing to regs
        if(reg_writesig && halfword == 0)
            case(dr)
                0: AX <= data_write;
                1: BX <= data_write;
                2: CX <= data_write;
                3: DX <= data_write;
                4: EX <= data_write;
                5: FX <= data_write;
                endcase
        else if(reg_writesig && halfword == 1)
            case(dr)
                //AX
                0: AXL <= data_write;
                1: AXH <= data_write;
                //BX
                2: BXL <= data_write;
                3: BXH <= data_write;
                //CX
                4: CXL <= data_write;
                5: CXH <= data_write;
                //DX
                6: DXL <= data_write;
                7: DXH <= data_write;
                //EX
                8: EXL <= data_write;
                9: EXH <= data_write;
                //FX
                10: FXL <= data_write;
                11: FXH <= data_write;

            endcase
        end

endmodule

module RAM(                     //memory
    input [6:0] addr,           //address
    input clk,                  //clock
    input [19:0] data_in,       //writing data
    input readsig,              //read signal
    input control,              //control signal
    input writesig,             //write signal
    output reg [19:0] data_out
); 

    reg [19:0] mem [0:64];  //64 words of 20 bits each

    always @(posedge clk)                   //writes data to specified address if NOT in read mode, 
        if(control && writesig && ~readsig) //and with input from control + write signal
            mem[addr] = data_in;
        
    always @(posedge clk)
        if(control && readsig && ~writesig) //reads data from specified address if NOT in write mode,
            data_out = mem[addr];           //and with input from control + read signal
    
endmodule

module ROM(
    input [19:0] rom_addr, //address
    input readsignal, //read signal
    input control, //control signal
    output reg [19:0] rom_data //data output
);

//case statement
always @(rom_addr or readsignal or control) begin
    case(rom_addr) 
        0: rom_data = 20'h000;   //TRAP 
        1: rom_data = 20'h015;   //NOP  
        2: rom_data = 20'h029;   //JMP  
        3: rom_data = 20'h03E;   //JMPZ 
        4: rom_data = 20'h053;   //JMPS 
        5: rom_data = 20'h068;   //JMPZS 
        6: rom_data = 20'h07D;   //LSTAT 
        7: rom_data = 20'h092;   //XSTAT 
        8: rom_data = 20'h0A7;   //NOT
        9: rom_data = 20'h0BC;   //AND
        10: rom_data = 20'h0D1;  //OR
        11: rom_data = 20'h0E6;  //XOR
        12: rom_data = 20'h0FB;  //SHFTR
        13: rom_data = 20'h110;  //SHFTL
        14: rom_data = 20'h125;  //ROTR
        15: rom_data = 20'h13A;  //ROTL
        16: rom_data = 20'h14F;  //SWAP
        17: rom_data = 20'h164;  //INC
        18: rom_data = 20'h179;  //DEC
        19: rom_data = 20'h18E;  //ADD
        20: rom_data = 20'h1A3;  //ADDC
        21: rom_data = 20'h1B8;  //SUB
        22: rom_data = 20'h1CD;  //SUBC
        23: rom_data = 20'h1E2;  //EQ
        24: rom_data = 20'h1F7;  //GT
        25: rom_data = 20'h20C;  //LT
        26: rom_data = 20'h221;  //GET
        27: rom_data = 20'h236;  //LET
        28: rom_data = 20'h24B;  //MRR
        29: rom_data = 20'h260;  //LDC
        30: rom_data = 20'h275;  //LDD
        31: rom_data = 20'h28A;  //LDI
        32: rom_data = 20'h29F;  //STD
        33: rom_data = 20'h2B4;  //STI
        34: rom_data = 20'h2C9;  //MUL -- extra credit
        35: rom_data = 20'h2DE;  //DIV -- extra credit

        //registers
        36: rom_data = 20'h2E0;  //SX -- status register
        37: rom_data = 20'h30A;  //IC -- instruction count
        38: rom_data = 20'h334;  //MA -- mem access count
        39: rom_data = 20'h35E;  //MC -- mem correction count
        40: rom_data = 20'h388;  //IS -- instruction segment
        41: rom_data = 20'h3B2;  //DS -- dynamic segment

    endcase
end

endmodule

module control_unit (                   //control unit, by tyler
    input [15:0] instruction, // Current instruction
    input clk,
    reset, 
    output reg fetch_enable,
    output reg decode_enable,
    output reg execute_enable,
    output reg memory_read_enable,
    output reg memory_write_enable,
    output reg reg_write_enable,
    output reg halt 
    // ... other control signals 
);

    reg [3:0] current_state; 

    reg [15:0] rom [0:255]; // 256 instructions, 16-bit control word
    initial begin
  // Fetch state
  //for (int i = 0; i < 256; i++) begin
    rom[0] = 16'b1000000000000000; // fetch_enable = 1
  //end

  // ADD instruction (opcode 0000)
  rom[16'h0001] = 16'b0100000000000000; // Decode state: decode_enable = 1
  rom[16'h0002] = 16'b0010000000000000; // Execute state: execute_enable = 1
  rom[16'h0003] = 16'b0000000010000000; // Write-back state: reg_write_enable = 1

  // LOAD instruction (opcode 0001)
  rom[16'h0011] = 16'b0100001000000000; // Decode, then Memory Read states
  rom[16'h0012] = 16'b0000000010000000; // Write-back state: reg_write_enable = 1

  // STORE instruction (opcode 0010)
  rom[16'h0021] = 16'b0100000100000000; // Decode, then Memory Write states
  rom[16'h0022] = 16'b0000000000000000; // No write-back for STORE

  // (all other opcodes)
  //for (int i = 0; i < 256; i++) begin
    //if (i[15:12] != 4'b0000 && i[15:12] != 4'b0001 && i[15:12] != 4'b0010) begin
      rom[16'hFF] = 16'b0000000000000001; // halt = 1
    //end
  //end
end

    // State transition logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= 4'b0000; // clear Fetch state on reset
            halt <= 1'b0; // Clear halt signal
        end else begin
            case (current_state)
                4'b0000: begin // Fetch
                    fetch_enable <= 1'b1;
                    current_state <= 4'b0001; // Next state: Decode
                end
                4'b0001: begin // Decode
                    decode_enable <= 1'b1;
                    // Determine next state based on instruction type (ALU, memory access, etc.)
                    case (instruction[15:12]) 
                        4'b0000: current_state <= 4'b0010; // ALU instruction, next: Execute
                        4'b0001: current_state <= 4'b0100; // Load instruction, next: Memory Read
                        4'b0010: current_state <= 4'b0110; // Store instruction, next: Memory Write
                        default: current_state <= 4'b1000; // (invalid instruction)
                    endcase
                end
                4'b0010: begin // Execute (ALU)
                    execute_enable <= 1'b1;
                    current_state <= 4'b0101; 
                end
                4'b0100: begin // Memory Read
                    memory_read_enable <= 1'b1;
                    current_state <= 4'b0101; 
                end
                4'b0110: begin // Memory Write
                    memory_write_enable <= 1'b1;
                    current_state <= 4'b0101; 
                end
                4'b0101: begin // Write-back
                    reg_write_enable <= 1'b1;
                    current_state <= 4'b0000; 
                end
                4'b1000: begin // halt state (invalid instruction)
                    halt <= 1'b1;
                end
                default: current_state <= 4'b0000; // Unexpected state, reset to Fetch
            endcase
        end
    end

    //  Control signal generation (from ROM output)
    always @(current_state, instruction) begin
        {fetch_enable, decode_enable, execute_enable, memory_read_enable, memory_write_enable, reg_write_enable, halt /* ... other control signals */ } = rom[instruction & 16'hFF];
    end

endmodule
