/*Module performs Ain + Bin, Ain - Bin, Ain & Bin
  and ~Bin deoending on the specification of ALUop*/
module ALU(Ain,Bin,ALUop,out,Z);
input [15:0] Ain, Bin;
input [1:0] ALUop;
output [15:0] out;
reg[15:0] out;
output [2:0] Z;
wire [15:0] Addsub_out;
reg sub;
wire[2:0] Z;
wire ovf;
//assign Z = (out == 16'b0000000000000000) ? 1'b1 : 1'b0;
AddSub #(16) add(Ain,Bin,sub,Addsub_out,ovf);
setflags flagset(out,Z,ovf);
//Checking value of ALUop and doing operation specified
always@(*)begin
   case(ALUop)
    2'b00: {sub,out}={1'b0,Addsub_out};
    2'b01: {sub,out}={1'b1,Addsub_out};
    2'b10: {sub,out} = {1'bx,Ain & Bin} ;
    2'b11: {sub,out} = {1'bx,~Bin};
    default: out = {1'bx,{16{1'bx}}};
   endcase
 end
endmodule



module AddSub(a,b,sub,s,ovf) ;
  parameter n = 8 ;
  input [n-1:0] a, b ;
  input sub ;           // subtract if sub=1, otherwise add
  output [n-1:0] s ;
  output ovf ;          // 1 if overflow
  wire c1, c2 ;         // carry out of last two bits
  wire ovf = c1 ^ c2 ;  // overflow if signs don't match

  // add non sign bits
  Adder1 #(n-1) ai(a[n-2:0],b[n-2:0]^{n-1{sub}},sub,c1,s[n-2:0]) ;
  // add sign bits
  Adder1 #(1)   as(a[n-1],b[n-1]^sub,c1,c2,s[n-1]) ;
endmodule


//Multi-bit adder taken from slide set 6
module Adder1(a,b,cin,cout,s);
 parameter n=8;
 input [n-1:0] a,b;
 input cin;
 output [n-1:0] s;
 output cout;
 wire [n-1:0]s;
 wire cout;

assign {cout,s} = a + b + cin;
endmodule 

//Sets negative,ovf, and zero fla
module setflags(in,out,ovf);
input [15:0] in;
output [2:0] out;
input ovf;
reg [2:0] out; 
always @(*) begin
casex(in) 
16'b0: out ={1'b1,1'b0,ovf} ;
16'b1xxx_xxxx_xxxx_xxxx: out = {1'b0,1'b1,ovf};
default : out = {1'b0,1'b0,ovf};
endcase
end
endmodule

