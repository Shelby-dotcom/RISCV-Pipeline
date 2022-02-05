`timescale 1ps / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/16/2021 06:41:11 PM
// Design Name: 
// Module Name: single_riscv
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module pipeline_riscv(clk,reset,interupt,result);
input clk,reset,interupt;
output [7:0] result;


wire PCSrcE,MemWriteM,ALUSrcE,RegWriteW;
wire [1:0]ResultSrcW,ImmSrcD;
wire [3:0]ALUControlE;
wire [6:0]op;
wire [2:0]funct3;
wire [6:0]funct7_5;
wire ZEROE;
wire [4:0] Rs1E,Rs2E,RdM,RdW,RdE,Rs1D,Rs2D;
wire [1:0]ForwardAE,ForwardBE;
wire RegWriteM;
wire StallF,StallD,FlushE,FlushD,int_pend,pri_mux_en,FlushM;
wire [1:0] ResultSrcE;

datapath D1 (clk,reset,PCSrcE,ResultSrcW,MemWriteM,ALUSrcE,ImmSrcD,RegWriteW,ALUControlE,
op,funct3,funct7_5,ZEROE,result,StallF,StallD,FlushD,FlushM,Rs1D,Rs2D,FlushE,RdE,Rs2E,Rs1E,
ForwardAE,ForwardBE,RdM,RdW,interupt,mret_flag,int_pend,pri_mux_en);

controller C1(clk,reset,op,funct3,funct7_5,ZEROE,PCSrcE,ResultSrcW,MemWriteM,ALUSrcE,ImmSrcD,
RegWriteW,ALUControlE,ResultSrcE,RegWriteM,FlushE,mret_flag);
HazardUnit HU1 (StallF,StallD,FlushD,Rs1D,Rs2D,FlushE,RdE,Rs2E,Rs1E,PCSrcE,ForwardAE,
ForwardBE,ResultSrcE[0],RdM,RegWriteM,RdW,RegWriteW,int_pend,pri_mux_en,FlushM);

endmodule

module address_generator(PCNext,clk,reset,StallF,PC);
input [31:0]PCNext;
input clk,reset,StallF;
output reg [31:0]PC;

always @(posedge clk) begin
    if (reset) begin
        PC<=0;
    end
    else if (StallF)
    begin
        PC<=PC;
    end
    else 
    begin
        
        PC <= PCNext;
    end
        
    
end




endmodule

module Adder(input   [31:0] a, b,
             output  [31:0] y);
     assign y = a + b;
endmodule

module ALU(
           input [31:0] A,B,  // ALU 8-bit Inputs
           input [3:0] ALU_Sel,// ALU Selection
           output [31:0] ALU_Out, // ALU 8-bit Output
           //output CarryOut, // Carry Out Flag
           output ZeroOut	// Zero Flag
    );
    
    reg [31:0] ALU_Result;
    //wire [32:0] tmp;
    assign ALU_Out = ALU_Result; // ALU out
    //assign tmp = {1'b0,A} + {1'b0,B};
    //assign CarryOut = (ALU_Result != 0); // Carryout flag
	  assign ZeroOut = (ALU_Result == 0) ;// Zero Flag
    always @(*)
    begin
        case(ALU_Sel)
        4'b0000: // Addition
           ALU_Result = A + B ;
        4'b0001: // Subtraction
           ALU_Result = A - B ;
        4'b0010: // Multiplication
           ALU_Result = A * B;
        4'b0011: // Division
           ALU_Result = A/B;
        4'b0100: // Logical shift left
           ALU_Result = A<<B;
         4'b0101: // Logical shift right
           ALU_Result = A>>B;
         4'b0110: // Rotate left
           ALU_Result = {A[30:0],A[31]};
         4'b0111: // Rotate right
           ALU_Result = {A[0],A[31:1]};
          4'b1000: //  Logical and
           ALU_Result = A & B;
          4'b1001: //  Logical or
           ALU_Result = A | B;
          4'b1010: //  Logical xor
           ALU_Result = A ^ B;
          4'b1011: //  Logical nor
           ALU_Result = ~(A | B);
          4'b1100: // Logical nand
           ALU_Result = ~(A & B);
          4'b1101: // Logical xnor
           ALU_Result = ~(A ^ B);
          4'b1110: // Less than comparison
           ALU_Result = (A<B)?32'd0:32'd1 ;
          4'b1111: // Equal comparison
            ALU_Result = (A==B)?32'd1:32'd0 ;
          default: ALU_Result = A + B ;
        endcase
    end

endmodule

module controller(clk,reset,op,funct3,funct7_5,ZEROE,PCSrcE,ResultSrcW,MemWriteM,ALUSrcE,ImmSrcD,RegWriteW,ALUControlE,
ResultSrcE,RegWriteM,FlushE,mret_flag);

input clk,reset;
output PCSrcE,MemWriteM,ALUSrcE;
output [1:0]ResultSrcW,ImmSrcD;
output [3:0]ALUControlE;
input [6:0]op;
input [2:0]funct3;
input [6:0]funct7_5;
input ZEROE;
input FlushE;

output [1:0] ResultSrcE;
output RegWriteM,RegWriteW,mret_flag;

wire [1:0] ResultSrcD,ResultSrcM;
wire MemWriteD,ALUSrcD,RegWriteD,MemWriteE,RegWriteE;
wire [3:0] ALUControlD;
wire BranchD,JumpD,BranchE,JumpE;
wire [1:0]ALUOp;

main_decoder MD1 (op, ResultSrcD, MemWriteD, BranchD,
 ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOp,mret_flag);
ALU_decoder ALD1 (op[5], funct3, funct7_5, ALUOp, ALUControlD);
ExecuteCtrReg DCR1 (clk,FlushE,ResultSrcD, MemWriteD, BranchD,
 ALUSrcD, RegWriteD, JumpD,ALUControlD,ResultSrcE, MemWriteE, BranchE,
 ALUSrcE, RegWriteE, JumpE,ALUControlE);
MemCtrReg ECR1 (clk,reset,RegWriteE,ResultSrcE,MemWriteE,RegWriteM,ResultSrcM,MemWriteM);
WriteCtrReg MCR1 (clk,reset,RegWriteM,ResultSrcM,RegWriteW,ResultSrcW);
assign PCSrcE = (BranchE & ZEROE ) | JumpE;

endmodule

module Data_Memory(input clk,input reset,output reg [31:0] Data_Out, input [31:0] Data_In, input [31:0] D_Addr, input wr);
		reg [31:0] Mem [255:0];			// Data Memory

	// Write your code here
    integer i;
    always @(negedge clk) begin
        if (reset) begin
            for (i =0 ;i<256 ;i=i+1 ) begin
                Mem[i]<= 32'h0;
                
            end
        end
        if (wr) begin
            Mem[D_Addr] <= Data_In;
        end
        Data_Out <= Mem[D_Addr];
        
    end





endmodule

module CSR(clk,interupt,mret_flag,PCE,int_pend,pri_mux_en,excep_pc);
input clk,interupt,mret_flag;
input [31:0]PCE;

output reg int_pend;
output pri_mux_en;
output reg [31:0]excep_pc;

reg [31:0] ret_addr;

always @(posedge interupt) begin
    int_pend <= 1'b1;
    ret_addr <= PCE;
    excep_pc <= 96;
end

assign pri_mux_en = int_pend | mret_flag;

always @(posedge clk) begin
    int_pend <= 1'b0;
end

always @(*) begin
    if (mret_flag) begin
        excep_pc <= ret_addr;
    end
    else begin
        excep_pc <= 96;
    end
end
endmodule

module ALU_decoder(opb5, funct3, funct7_5, ALUOp, ALUControlD);
input opb5;
input [2:0]funct3;
input [6:0]funct7_5;
input [1:0] ALUOp;
output reg [3:0]ALUControlD;

wire Rtypesub;
assign Rtypesub = funct7_5[5] & opb5;

always @(*) begin
    casex (ALUOp)
        2'b00: ALUControlD = 4'b0000; //lw and sw
        2'b01: case (funct3)
            3'b000:  ALUControlD = 4'b0001; //branch equal
            3'b001:  ALUControlD = 4'b1111; //branch not equal
            3'b100:  ALUControlD = 4'b1110; //branch less than
            default: ALUControlD = 4'b0000;
        endcase 
        
        
        2'b10: casex (funct3)
            3'b000: case (Rtypesub)
                0: ALUControlD = 4'b0000;  //add
                1: ALUControlD =  4'b0001;  //sub

                default: ALUControlD =4'b0000;
            endcase 
            3'b100: ALUControlD = 4'b0011; //div
            3'b111: ALUControlD = 4'b1000; //and
            3'b110: ALUControlD = 4'b1001; //or
            3'b100: ALUControlD = 4'b1010; //xor
            3'b001: ALUControlD = 4'b0100; //sll
            3'b101: ALUControlD = 4'b0101; //srl
            //3'b101: ALUControlD = 4'b1110; //sgt
            default: ALUControlD = 3'bxxx;
        endcase
        default: ALUControlD = 3'bxxx;
    endcase
    
end
endmodule

module datapath(clk,reset,PCSrcE,ResultSrcW,MemWriteM,
ALUSrcE,ImmSrcD,RegWriteW,ALUControlE,op,funct3,funct7_5,ZEROE,result,
StallF,StallD,FlushD,FlushM,Rs1D,Rs2D,FlushE,RdE,Rs2E,Rs1E,
ForwardAE,ForwardBE,RdM,RdW,interupt,mret_flag,int_pend,pri_mux_en);

input clk,reset;
input PCSrcE,MemWriteM,ALUSrcE,RegWriteW;
input [1:0]ResultSrcW,ImmSrcD;
input [3:0]ALUControlE;
input [1:0]ForwardAE,ForwardBE;
input StallF,StallD,FlushE,FlushD,FlushM;
input interupt,mret_flag;

output [6:0]op;
output [2:0]funct3;
output [6:0]funct7_5;
output ZEROE;
output [7:0]result;
output [4:0] Rs1D,Rs2D,Rs1E,Rs2E,RdW,RdE,RdM;


wire [31:0] instructionF,instructionD;
wire [31:0] PCFprime,PCF,PCPlus4F,PCTargetE,PCD,PCPlus4D,PCE,PCPlus4E,PCPlus4M,PCPlus4W;
wire [4:0] A1,A2,RdD;
wire [24:0]extend_in;
wire [31:0]SrcAD,WriteDataD,ResultW,ReadDataM,SrcAE,WriteDataE,WriteDataM,ReadDataW,RD1E,RD2E;
wire [31:0]SrcBE,ImmExtD,ALUResultE,ImmExtE,ALUResultM,ALUResultW;
wire [31:0]result_reg;
wire [31:0]Prior_in;
output pri_mux_en;
wire [31:0]excep_pc;
wire mret_flag;
output int_pend;


mux2x1 M1 (PCPlus4F,PCTargetE,PCSrcE,Prior_in);
mux2x1 PM1 (Prior_in,excep_pc,pri_mux_en,PCFprime);
address_generator AG1 (PCFprime,clk,reset,StallF,PCF);
Instruction_Memory IM1 (PCF,instructionF);
Adder Add1 (PCF,4,PCPlus4F);
DecodeReg FR1 (clk,FlushD,StallD,instructionF,PCF,PCPlus4F,instructionD,PCD,PCPlus4D);

Instruction_fetch IF1 (instructionD,reset,clk,op,funct3,funct7_5,A1,A2,RdD,Rs1D,Rs2D,extend_in);
register_file RF1 (clk,reset,SrcAD, WriteDataD, ResultW, A1, A2, RdW, RegWriteW,result_reg);
Sign_extend SE1 (extend_in,ImmSrcD,ImmExtD);
ExecuteReg DR1 (clk,FlushE,SrcAD, WriteDataD,PCD,Rs1D,Rs2D,RdD,ImmExtD,PCPlus4D,RD1E, RD2E,PCE,Rs1E,Rs2E,RdE,ImmExtE,PCPlus4E);

CSR mod1 (clk,interupt,mret_flag,PCE,int_pend,pri_mux_en,excep_pc);
mux3x1 M2 (RD1E,ResultW,ALUResultM,ForwardAE,SrcAE);
mux3x1 M3 (RD2E,ResultW,ALUResultM,ForwardBE,WriteDataE);
mux2x1 M4 (WriteDataE,ImmExtE,ALUSrcE,SrcBE);
ALU alu1 (SrcAE,SrcBE,ALUControlE,ALUResultE,ZEROE);
Adder Add2 (PCE,ImmExtE,PCTargetE);
MemReg ER1 (clk,FlushM,ALUResultE,WriteDataE,RdE,PCPlus4E,ALUResultM,WriteDataM,RdM,PCPlus4M);

Data_Memory DM1 (clk,reset,ReadDataM,WriteDataM,ALUResultM,MemWriteM);
WriteReg MR1 (clk,reset,ALUResultM,ReadDataM,RdM,PCPlus4M,ALUResultW,ReadDataW,RdW,PCPlus4W);

mux3x1 M5 (ALUResultW,ReadDataW,PCPlus4W,ResultSrcW,ResultW);




assign result=result_reg[7:0];


endmodule

module ExecuteCtrReg(clk,FlushE,ResultSrcD, MemWriteD, BranchD,
 ALUSrcD, RegWriteD, JumpD,ALUControlD,ResultSrcE, MemWriteE, BranchE,
 ALUSrcE, RegWriteE, JumpE,ALUControlE);

input clk,FlushE;
input [1:0] ResultSrcD;
input MemWriteD, BranchD,ALUSrcD, RegWriteD, JumpD;
input [3:0] ALUControlD; 

output reg[1:0] ResultSrcE;
output reg MemWriteE, BranchE,ALUSrcE, RegWriteE, JumpE;
output reg [3:0] ALUControlE; 

always @(posedge clk) begin
    if (FlushE) begin
        ResultSrcE <= 0;
        MemWriteE <= 0; 
        BranchE <= 0;
        ALUSrcE <= 0; 
        RegWriteE <= 0;
        JumpE <= 0;
        ALUControlE <= 0;
    end
    else 
    begin
        
        ResultSrcE <= ResultSrcD;
        MemWriteE <= MemWriteD; 
        BranchE <= BranchD;
        ALUSrcE <= ALUSrcD; 
        RegWriteE <= RegWriteD;
        JumpE <= JumpD;
        ALUControlE <= ALUControlD;
    end
end
endmodule

module HazardUnit(StallF,StallD,FlushD,Rs1D,Rs2D,FlushE,RdE,Rs2E,Rs1E,
PCSrcE,ForwardAE,ForwardBE,
ResultSrcE0,RdM,RegWriteM,RdW,RegWriteW,int_pend,pri_mux_en,FlushM );
    input [4:0] Rs1E,Rs2E,RdW,RdM,RdE,Rs1D,Rs2D;
    input RegWriteM,RegWriteW;
    input ResultSrcE0;
    input PCSrcE,int_pend,pri_mux_en;

    output reg FlushE,FlushD,FlushM;
    output reg StallF,StallD;
    output reg [1:0]ForwardAE,ForwardBE;

    reg lwStall;

//Forward A
always @(*) begin
    if (   ((Rs1E==RdM) && RegWriteM )  &&  (Rs1E != 0)    )
    begin
        ForwardAE <= 2'b10;
    end
    else if ( ((Rs1E==RdW) && RegWriteW )  &&  (Rs1E != 0) ) begin
        ForwardAE <= 2'b01;
    end
    else begin
        ForwardAE <= 2'b00;
    end
end
    
        
always @(*) begin
    //Forward B
    if (   ((Rs2E==RdM) && RegWriteM )  &&  (Rs2E != 0)    )
    begin
        ForwardBE <= 2'b10;
    end
    else if ( ((Rs2E==RdW) && RegWriteW )  &&  (Rs2E != 0) ) begin
        ForwardBE <= 2'b01;
    end
    else begin
        ForwardBE <= 2'b00;
    end

end
    
    //Stalling
    always @(*) begin
         lwStall <= ResultSrcE0 & ( (Rs1D == RdE)  | (Rs2D == RdE) );
         StallD <= lwStall;
         FlushE <= lwStall | PCSrcE | int_pend ;
         StallF <= lwStall; 
         FlushD <= PCSrcE | int_pend | pri_mux_en;
         FlushM <= int_pend;
    end
    
        

    
    

    
endmodule

module DecodeReg(clk,FlushD,StallD,instructionF,PCF,PCPlus4F,instructionD,PCD,PCPlus4D);
input clk,FlushD,StallD;
input [31:0] instructionF,PCF,PCPlus4F;
output reg [31:0]  instructionD,PCD,PCPlus4D;


always @(posedge clk) begin
    if (FlushD) begin
        instructionD <= 0;
        PCD <= 0;
        PCPlus4D <=0;
    end
    else if (StallD)
    begin
        instructionD <= instructionD;
        PCD <= PCD;
        PCPlus4D <= PCPlus4D;
    end
    else
    begin
        instructionD <= instructionF;
        PCD <= PCF;
        PCPlus4D <= PCPlus4F;
    end
end

endmodule

module ExecuteReg(clk,FlushE,SrcAD, WriteDataD,PCD,Rs1D,Rs2D,RdD,ImmExtD,PCPlus4D,RD1E, RD2E,PCE,Rs1E,Rs2E,RdE,ImmExtE,PCPlus4E);
input clk,FlushE;
input [31:0]SrcAD, WriteDataD,PCD,ImmExtD,PCPlus4D;
input [4:0] RdD,Rs1D,Rs2D;

output reg [31:0]RD1E, RD2E,PCE,ImmExtE,PCPlus4E;
output reg [4:0] RdE,Rs1E,Rs2E;

always @(posedge clk) begin
    if (FlushE) begin
        RD1E <= 0;
        RD2E <= 0;
        PCE <= 0;
        RdE <= 0;
        Rs1E <= 0;
        Rs2E <= 0;
        ImmExtE <= 0;
        PCPlus4E <= 0;
    end
    else 
    begin
        RD1E <= SrcAD;
        RD2E <= WriteDataD;
        PCE <= PCD;
        RdE <= RdD;
        Rs1E <= Rs1D;
        Rs2E <= Rs2D;
        ImmExtE <= ImmExtD;
        PCPlus4E <= PCPlus4D;
    end
end
endmodule

module Instruction_Memory(input [31:0] pc,output [31:0] instruction);
// Write your code here
    reg [31:0] instruct [2825:0];
    integer i=0;
    initial begin
        $readmemh("Verify.mem",instruct);
    end
    assign instruction = instruct [pc/4];
endmodule

module main_decoder(op, ResultSrcD, MemWriteD, BranchD,
 ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOp,mret_flag);

input [6:0] op;
output [1:0] ResultSrcD;
output MemWriteD;
output BranchD, ALUSrcD;
output RegWriteD, JumpD;
output [1:0] ImmSrcD;
output [1:0] ALUOp;
output mret_flag;

reg [11:0] controls;
assign {RegWriteD, ImmSrcD, ALUSrcD, MemWriteD,
ResultSrcD, BranchD, ALUOp, JumpD,mret_flag} = controls;
//assign mret_flag = op == 7'b1110011; 
always @(*) begin
casex (op)
// RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump_mretflag
7'b0000011: controls = 12'b1_00_1_0_01_0_00_0_0; // lw
7'b0100011: controls = 12'b0_01_1_1_00_0_00_0_0; // sw
7'b0110011: controls = 12'b1_xx_0_0_00_0_10_0_0; // R–type
7'b1100011: controls = 12'b0_10_0_0_00_1_01_0_0; // branch
7'b0010011: controls = 12'b1_00_1_0_00_0_10_0_0; // I–type ALU
7'b1101111: controls = 12'b1_11_0_0_10_0_00_1_0; // jal
7'b1110011: controls = 12'b0_00_0_0_00_0_00_0_1; //CSR mret 

    default: controls = 12'b0_00_0_0_00_0_00_0; // ??? 
endcase
end

endmodule

module mux3x1(input  [31:0] d0, d1, d2,
                      input  [1:0]       s,
                      output [31:0] y);
     assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule

module Instruction_fetch(
input [31:0] instruction, 
input rst, 
input clk,
output reg [6:0]op_code,
output reg [2:0]funct3,
output reg [6:0]funct7_5,
output reg [4:0]A1,
output reg [4:0]A2,
output reg [4:0]A3,
output reg [4:0]Rs1D,
output reg [4:0] Rs2D,
output reg [24:0]extend_in

    );
// Write your code here
    always @(*) begin
        op_code <= instruction[6:0];
        funct3 <= instruction[14:12];
        funct7_5 <= instruction[31:25];
        A1 <= instruction[19:15];
        A2 <= instruction[24:20];
        A3 <= instruction[11:7];
        Rs1D <= instruction [19:15];
        Rs2D <= instruction [24:20];        
        extend_in <= instruction[31:7];
        
    end
endmodule

module MemCtrReg(clk,reset,RegWriteE,ResultSrcE,MemWriteE,RegWriteM,ResultSrcM,MemWriteM);

input clk,reset;
input RegWriteE,MemWriteE;
input [1:0]ResultSrcE;

output reg RegWriteM,MemWriteM;
output reg [1:0]ResultSrcM;

always @(posedge clk) begin
    if (reset) begin
        RegWriteM <= 0;
        ResultSrcM <= 0;
        MemWriteM <= 0;
    end
    else 
    begin
        RegWriteM <= RegWriteE;
        ResultSrcM <= ResultSrcE;
        MemWriteM <= MemWriteE;
        
    end
end


endmodule

module MemReg(clk,FlushM,ALUResultE,WriteDataE,RdE,PCPlus4E,ALUResultM,WriteDataM,RdM,PCPlus4M);
input clk,FlushM;
input [31:0] ALUResultE,WriteDataE,PCPlus4E;
input [4:0] RdE;

output reg [31:0] ALUResultM,WriteDataM,PCPlus4M ;
output reg [4:0]  RdM;

always @(posedge clk) begin
    if (FlushM) begin
        ALUResultM <= 0;
        WriteDataM <= 0;
        RdM <= 0;
        PCPlus4M <= 0;
    end
    else 
    begin
        
        ALUResultM <= ALUResultE;
        WriteDataM <= WriteDataE;
        RdM <= RdE;
        PCPlus4M <= PCPlus4E;
    end
end

endmodule

module mux2x1 (input [31:0] d0, d1,
               input              s,
               output [31:0] y);
     assign y = s ? d1 : d0;
endmodule


module Sign_extend (input  		[31:7] instr,
                    input  		[1:0]  immsrc,
                    output reg 	[31:0] immext   );
    always @(*)begin
        
    casex(immsrc)
         // I−type
    2'b00:     immext = {{20{instr[31]}}, instr[31:20]};
		 // S−type (stores)
    2'b01:     immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
         // B−type (branches)
    2'b10:      immext = {{20{instr[31]}}, instr[7],  instr[30:25], instr[11:8], 1'b0};                          // J−type (jal)
		// J−type (branches)
	2'b11:      immext = {{12{instr[31]}}, instr[19:12],  instr[20], instr[30:21], 1'b0};
           
	default: 	immext = 32'bx; // undefined
    endcase
    end
endmodule

module register_file(clk,reset,Port_A, Port_B, Din, Addr_A, Addr_B, Addr_Wr, wr,result_reg);
			output reg [31:0] Port_A, Port_B;			// Data to be provided from register to execute the instruction
            output [31:0]result_reg;
			input [31:0] Din;						// Data to be loaded in the register
			input [4:0] Addr_A, Addr_B, Addr_Wr;	// Address (or number) of register to be written or to be read
			input wr,clk,reset;								// input wr flag input
			reg [31:0] Reg_File [31:0];				// Register file
            integer i=0;
// Write your code here
    
        always @(negedge clk) begin
            
            
            if(reset)begin
                for(i=0;i<32;i=i+1)
                    Reg_File[i]<=32'h0;
            end else if (wr && (Addr_Wr!=0)) begin
            Reg_File[Addr_Wr]=Din;
            end
 
        
    end
    always @(*) begin
        Reg_File[0]=0;
        Port_A=Reg_File[Addr_A];
        Port_B=Reg_File[Addr_B];

    end
    
    assign result_reg = Reg_File[5];
endmodule


module WriteCtrReg(clk,reset,RegWriteM,ResultSrcM,RegWriteW,ResultSrcW);

input clk,reset;
input RegWriteM;
input [1:0] ResultSrcM;

output reg RegWriteW;
output reg [1:0] ResultSrcW;

always @(posedge clk) begin
    if (reset) begin
        RegWriteW <= 0;
        ResultSrcW <= 0;
    end
    else 
    begin
        RegWriteW <= RegWriteM;
        ResultSrcW <= ResultSrcM;
        
    end
end


endmodule

module WriteReg(clk,reset,ALUResultM,ReadDataM,RdM,PCPlus4M,ALUResultW,ReadDataW,RdW,PCPlus4W);
input clk,reset;
input [31:0] ALUResultM,ReadDataM,PCPlus4M;
input [4:0] RdM;

output reg [31:0]ALUResultW,ReadDataW,PCPlus4W;
output reg [4:0] RdW;

always @(posedge clk) begin
    if (reset) begin
        ALUResultW <= 0;
        ReadDataW <= 0;
        RdW <= 0;
        PCPlus4W <= 0;
    end
    else 
    begin
        
        ALUResultW <= ALUResultM;
        ReadDataW <= ReadDataM;
        RdW <= RdM;
        PCPlus4W <= PCPlus4M;
    end
end

endmodule
