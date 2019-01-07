//Regfile for Simple RISC machine written by Samuel Kinneman
module regfile(data_in,writenum,write,readnum,clk,data_out);
input [15:0] data_in;
input [2:0] writenum, readnum;
input write, clk;
output [15:0] data_out;
wire [15:0] data_out;
//Outputs of decoders 
wire[7:0] write_num_dec, read_num_dec, write_load;
//Outputs of Registers
wire[15:0] R0,R1,R2,R3,R4,R5,R6,R7;

//Instantiating both decoders
Dec #(3,8) mux_dec(readnum,read_num_dec);
Dec #(3,8) write_dec(writenum,write_num_dec);

//Logic for load on load register
assign write_load = {8{write}} & write_num_dec;

//Instantiate Multiplexer
Mux8a mux(R7,R6,R5,R4,R3,R2,R1,R0,read_num_dec,data_out);

//Instantiate Load enabled registers
regload R_0(data_in,write_load[0],clk,R0);
regload R_1(data_in,write_load[1],clk,R1);
regload R_2(data_in,write_load[2],clk,R2);
regload R_3(data_in,write_load[3],clk,R3);
regload R_4(data_in,write_load[4],clk,R4);
regload R_5(data_in,write_load[5],clk,R5);
regload R_6(data_in,write_load[6],clk,R6);
regload R_7(data_in,write_load[7],clk,R7);

endmodule

//Load enable register
module regload(in,load,clk,out);
input[15:0] in;
input load, clk;
output[15:0] out;
wire[15:0] flip_in, out;
//Instantiating flip flop
vDFF#(16) ff(clk,flip_in,out);
//Load in mux
assign flip_in = load ? in : out;
endmodule


//Code for decoder taken from slide set
module Dec(a,b);
parameter n=2;
parameter m=4;

input [n-1:0] a;
output [m-1:0] b;

wire [m-1:0] b = 1 << a ;
endmodule


//Using Mux style from slide set, except 8 inputs 16 bit inputs and outputs
module Mux8a(a7,a6,a5,a4,a3,a2,a1,a0,sel,out);
 input[15:0] a7,a6,a5,a4,a3,a2,a1,a0;
 input[7:0] sel;
 output[15:0] out;
 reg[15:0] out;
 
 //Checking which register was loaded
 always @(*)begin
   case(sel)
    8'b00000001: out = a0;
    8'b00000010: out = a1;
    8'b00000100: out = a2;
    8'b00001000: out = a3;
    8'b00010000: out = a4;
    8'b00100000: out = a5;
    8'b01000000: out = a6;
    8'b10000000: out = a7;
    default: out = {16{1'bx}};
   endcase
 end
endmodule

