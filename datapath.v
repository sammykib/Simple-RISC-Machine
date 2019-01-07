/*Datapath module written by Samuel Kinneman, all of the modules instantiated are named to match the naming
  shown on the diagram of datapath in lab 5.
*/
module datapath(clk,readnum,vsel,loada,loadb,shift,asel,bsel,ALUop,loadc,loads,writenum,write,mdata,Z_out,datapath_out,PC,sximm8,sximm5);
input clk, write, loada, loadb, asel, bsel, loadc, loads;
input [15:0] mdata, sximm8,sximm5;
input [8:0] PC;
input [2:0] readnum, writenum,vsel;
input [1:0] shift, ALUop;
output [15:0] datapath_out;
output[2:0] Z_out;
wire[15:0] regfile_out, regfile_in, reg_A_out, reg_B_out, shift_out, Ain_mux, Bin_mux, ALU_out, datapath_out;
wire[2:0] Z;

//Instntiating shifter, ALU, and regfile
regfile REGFILE(regfile_in, writenum, write, readnum, clk,regfile_out);
ALU alu_dp(Ain_mux,Bin_mux,ALUop,ALU_out,Z);
shifter shifer_dp(reg_B_out,shift,shift_out);

//Instantiating all load enable registers, names of each register indicate what they are on diagram of handout
regload reg_A(regfile_out,loada,clk,reg_A_out);
regload reg_B(regfile_out,loadb,clk,reg_B_out);
regload reg_C(ALU_out,loadc,clk,datapath_out);
//Made a new module because I made regload only for 15 bits
statusregister status_reg(Z,loads,clk,Z_out);

//Assigning muxes for all of datapath
assign Ain_mux = asel ? {16{1'b0}} : reg_A_out;
assign Bin_mux = bsel ? sximm5 : shift_out ;
//assign regfile_in = vsel ? datapath_in : datapath_out;
//Vsel Mux with added inputs from Lab 6
Mux4a vsel_mux(mdata,sximm8,{7'b0,PC},datapath_out,vsel,regfile_in);

endmodule

//Status register
module statusregister(in,load,clk,out);
input[2:0] in;
input load, clk;
output[2:0] out;
wire[2:0] flip_in, out;
//Instantiating flip flop
vDFF#(3) ff(clk,flip_in,out);
//Load in mux
assign flip_in = load ? in : out;
endmodule


//Using Mux style from slide set, except 4 inputs 16 bit inputs and outputs
module Mux4a(a3,a2,a1,a0,sel,out);
 input[15:0] a3,a2,a1,a0;
 input[2:0] sel;
 output[15:0] out;
 reg[15:0] out;
 
 //Checking which register was loaded
 always @(*)begin
   case(sel)
    3'b001: out = a0;
    3'b010: out = a1;
    3'b011: out = a2;
    3'b100: out = a3;
    default: out = {16{1'bx}};
   endcase
 end
endmodule

module vDFF(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;

  always @(posedge clk)
    Q <= D;
endmodule