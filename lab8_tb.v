module lab8_tb();
reg CLOCK_50;
reg [3:0] KEY;
reg [9:0] SW;
wire [9:0] LEDR;
wire [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

lab8_top DUT(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);

initial begin
  CLOCK_50 = 0; #5;
  forever begin
  CLOCK_50 = 1; #5;
  CLOCK_50 = 0; #5;
  end
end 

initial begin
SW[7:0] = 8'b0000_0011; 
#10;
KEY[1] = 1'b0;
#10;
KEY[1] = 1'b1;
#8000;












$stop;
end
endmodule
