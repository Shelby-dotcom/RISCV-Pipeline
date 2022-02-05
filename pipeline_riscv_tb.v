`timescale 1ps / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/24/2021 09:24:57 PM
// Design Name: 
// Module Name: single_riscv_tb
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


module pipeline_riscv_tb();

reg clk;
reg reset;
wire [7:0]result;
reg interupt;


pipeline_riscv DUT (clk,reset,interupt,result);

initial clk =0;
always #5 clk = ~clk;

initial begin
    reset<= 1;
    interupt <= 0;
    #100
    reset<= 0;
    interupt <= 0;
    #40;
    reset<= 0;
    interupt <= 0;
    #50;


    

    
end
endmodule
