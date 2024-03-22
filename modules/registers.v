module registers (                  //code for dynamic GP registers, by jaiden
    input clk, reset,               //clock signal, reset signal
    input writesig,                 //write signal
    input readsig,                  //read signal

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
        if(readsig && halfword == 0)
            case(sr)
                0: data_read = AX;
                1: data_read = BX;
                2: data_read = CX;
                3: data_read = DX;
                4: data_read = EX;
                5: data_read = FX;
                default: data_read = 20'b0;
            endcase
        else if(readsig && halfword == 1)
            case(sr)
                0: data_read = AXL;
                1: data_read = AXH;
                2: data_read = BXL;
                3: data_read = BXH;
                4: data_read = CXL;
                5: data_read = CXH;
                6: data_read = DXL;
                7: data_read = DXH;
                8: data_read = EXL;
                9: data_read = EXH;
                10: data_read = FXL;
                11: data_read = FXH;
            endcase
        end

        always @(posedge clk) begin                 //switch statement for writing to regs
        if(writesig && halfword == 0)
            case(dr)
                0: AX <= data_write;
                1: BX <= data_write;
                2: CX <= data_write;
                3: DX <= data_write;
                4: EX <= data_write;
                5: FX <= data_write;
                endcase
        else if(writesig && halfword == 1)
            case(dr)
                0: AXL <= data_write;
                1: AXH <= data_write;
                2: BXL <= data_write;
                3: BXH <= data_write;
                4: CXL <= data_write;
                5: CXH <= data_write;
                6: DXL <= data_write;
                7: DXH <= data_write;
                8: EXL <= data_write;
                9: EXH <= data_write;
                10: FXL <= data_write;
                11: FXH <= data_write;

            endcase
        end

endmodule
