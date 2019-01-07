module shifter(in,shift,sout);
input [15:0] in;
input [1:0] shift;
output [15:0] sout;
reg [15:0] sout;
//Checking value of shift to determuine correct operation
always@(*)begin
 case(shift)
  2'b00: sout = in;
  2'b01: sout = {in[14:0], 1'b0};
  2'b10: sout = {1'b0,{in[15:1]}};
  2'b11: sout = {{in[15]},{in[15:1]}};
 default: sout = {16{1'bx}};
 
endcase 
end
endmodule
