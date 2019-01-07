module lab8_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);
input CLOCK_50;
input [3:0] KEY;
input [9:0] SW;
output [9:0] LEDR;
output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

wire write,write_comp_out,read_comp_out, msel,load_led,check_addr_out,check_cmd_out;
wire check_addr2_out,check_write_out;
wire [15:0] mdata,out,dout;
wire [8:0] mem_addr;
wire [1:0] mem_cmd;
wire[2:0] led_on;
//Assigning value for LEDR[8]
updateLED LEDR_up(LEDR[8],led_on);
//instantiating cpu
cpu CPU(CLOCK_50,~KEY[1],mdata,out,mem_cmd,mem_addr,led_on);

//Instantiating RAM
RAM #(16,9) MEM(CLOCK_50,mem_addr,mem_addr,write,out,dout);

//Assigning value of write
assign write = (write_comp_out & msel);
//Instantiating quality comparators for memory
EqComp #(2)Write_comp(mem_cmd,2'b11,write_comp_out);
EqComp #(2)Read_comp(mem_cmd,2'b00,read_comp_out);
EqComp #(1)HO_bits(mem_addr[8],1'b0,msel);

//tri state driver logic
assign mdata = (msel & read_comp_out) ? dout : {16{1'bz}};

//Rest is I/O device part
//tri-state driver for Input and its accompanying circuit
assign mdata = (check_cmd_out & check_addr_out) ? { {8{1'b0}},SW[7:0]} : {16{1'bz}};
EqComp #(2) check_cmd(mem_cmd,2'b00,check_cmd_out);
EqComp #(9) check_addr(mem_addr,9'b101000000,check_addr_out);

//Logic for output device circuit
LEvdFF #(8) output_register(out[7:0],load_led,CLOCK_50,LEDR[7:0]);
EqComp #(2) check_write(mem_cmd,2'b11,check_write_out);
EqComp #(9) check_addr2(mem_addr,9'b100000000,check_addr2_out);
//Logic for loading Leds
assign load_led = check_addr2_out & check_write_out;

endmodule 

// equality comparator
module EqComp(a, b, eq) ;
  parameter k=8;
  input  [k-1:0] a,b;
  output eq;
  wire   eq;

  assign eq = (a==b) ;
endmodule

//Module for updating LEDR8
module updateLED(LEDR,mdata);
input[2:0] mdata;
output LEDR;
reg LEDR;

always@(*)begin
case(mdata)
3'b111: LEDR = 1'b1; 
default : LEDR = 1'b0;
endcase
end
endmodule
