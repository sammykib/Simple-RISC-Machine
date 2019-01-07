module cpu(clk,reset,mdata,out,mem_cmd,mem_addr,led_on);
input clk, reset;
input [15:0] mdata;
output [8:0] mem_addr;
output [1:0] mem_cmd;
output [15:0] out;
output [2:0] led_on;

wire[15:0] IR_out, sximm5,sximm8,mdata,out;
wire [1:0] ALUop, op, shift,nsel,mem_cmd,add_sximm8;
wire [2:0] opcode,readnum,writenum,Z_out, vsel,cond,led_on;
wire  write, loada, loadb, asel, bsel, loadc, loads, load_ir, load_pc, reset_pc, load_addr,addr_sel, N, V, Z;
wire[8:0] next_pc, PC, mem_addr,data_add_out, PC_mux_out;

//Assigning values of Negative, Overflow, and Zero
assign N = Z_out[1];
assign V = Z_out[0];
assign Z = Z_out[2];

//Instantiating Data address reg
LEvdFF #(9) data_address(out[8:0],load_addr,clk,data_add_out);

//Instantiating Instruction register
LEvdFF #(16) instruct_reg(mdata,load_ir,clk,IR_out);

//Instantiating Program counter register
LEvdFF #(9) pc_counter_reg(PC_mux_out,load_pc,clk,PC);

//PC MUX
assign next_pc = reset_pc ? {9{1'b0}} : (PC + 1);

//mux that chooses to make PC = Rd or equal to added with sximm8
pc_adder PC_branch(next_pc,sximm8[8:0],add_sximm8,PC_mux_out,out[8:0]);

//Address sel mux
assign mem_addr = addr_sel ? PC : data_add_out;

//Instantiating Instruction Decoder
instruct_dec IR_dec(IR_out,shift,sximm5,sximm8,ALUop,opcode,op,nsel,readnum,writenum,cond);

//Instantiating state machine
state_machine FSM(clk,reset,write,opcode,op,asel,bsel,vsel,nsel,loada,loadb,loadc,loads,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,cond,add_sximm8, N, V, Z,led_on);

//Instantiating Datapath
datapath DP(clk,readnum,vsel,loada,loadb,shift,asel,bsel,ALUop,loadc,loads,writenum,write,mdata,Z_out,out,PC,sximm8,sximm5);

endmodule

//Module for state machine to be instantiated in cpu module
module state_machine(clk,reset,write,opcode,op,asel,bsel,vsel,nsel,loada,loadb,loadc,loads,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,cond,add_sximm8, N, V, Z,led_on);
input clk,reset,N, V, Z;
input[1:0] op; //load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd
input[2:0] opcode,cond;
output asel,bsel,loada,loadb,loadc,loads,write,load_ir,load_pc,reset_pc,load_addr,addr_sel;
output [1:0] nsel,mem_cmd,add_sximm8;
output [2:0] vsel,led_on;
//Wires for states
wire[4:0] present_state,state_next_reset;
reg[4:0] next_state;
reg asel,bsel,loada,loadb,loadc,loads,write,load_ir,load_pc,reset_pc,load_addr,addr_sel;
reg [1:0] nsel,mem_cmd,add_sximm8;
reg [2:0] vsel;
wire[2:0] led_on;
wire[4:0] p;

//p is the signal the autograder wants to know
assign p = present_state ;
//Constants
`define MREAD 2'b00
`define MWRITE 2'b11
`define ADDsx8 2'b01
`define NOTHING 2'b00
`define EQRD 2'b11

//States reset, If1,IF2,UpdatePC
`define IF1 5'b00001
`define IF2_x 5'b01010
`define Reset 5'b01011 
`define UpdatePC 5'b01100

//MOV states
`define DEC 5'b00000 
`define MOV_im8 5'b00010
`define MOV_rmb 5'b00011
`define SHIFT_rmb 5'b00100
//This state can be resused
`define WRITE_REG 5'b00101

//ALU states 
 `define mov_A 5'b00111
 `define mov_B 5'b00110
 `define ALU_AB 5'b01000
 `define mov_nB 5'b01001

//Store and Load states
 `define STR_Addr 5'b01101
 `define STR_value 5'b01110
 `define STR_value2 5'b01111 
 `define HALT 5'b10000

//State for setting PC
 `define set_PC 5'b10101

//assigning led_on for ledr8
assign led_on = (present_state == `HALT)? 3'b111:3'b000;

//Reset Logic MUX
assign state_next_reset = reset ? `Reset : next_state;
//Flip flop for state
vDFF #(5) STATE(clk,state_next_reset,present_state);


//Begining of always block with casex 
always@(*)begin
	casex({present_state,op,opcode,cond, N, V, Z})
         //IF, uodatePC and reset states
{`Reset,2'bxx,3'bxxx,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8}={`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b1,1'b0,1'b0,2'b10,`NOTHING}; //load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd(2)
{`IF1,2'bxx,3'bxxx,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8}={`IF2_x,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,`MREAD,`NOTHING};
{`IF2_x,2'bxx,3'bxxx,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8}={`UpdatePC,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b1,1'b0,1'b0,1'b0,1'b1,`MREAD,`NOTHING};
{`UpdatePC,2'bxx,3'bxxx,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8}={`DEC,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b0,2'b10,`NOTHING};

	 //States for Mov instructions in decode
{`DEC,2'b10,3'b110,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`MOV_im8,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
{`DEC,2'b00,3'b110,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`MOV_rmb,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
 	 
	 //States for ALU in decode
{`DEC,2'b11,3'b101,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`mov_nB,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
{`DEC,2'bxx,3'b101,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`mov_A,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};

	//States for decode added in Lab 7 (LDR,STR)
{`DEC,2'b00,3'b011,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`mov_A,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
{`DEC,2'b00,3'b100,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`mov_A,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};


	//Decode states for branch instructions, two for each except for uncond branch
     //B instruction
{`DEC,2'b00,3'b001,3'b000,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
    //BEQ instructions
{`DEC,2'b00,3'b001,3'b001,2'bxx,1'b1}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
{`DEC,2'b00,3'b001,3'b001,2'bxx,1'b0}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
   //BNE instructions
{`DEC,2'b00,3'b001,3'b010,2'bxx,1'b0}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
{`DEC,2'b00,3'b001,3'b010,2'bxx,1'b1}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
  //BLT instructions
{`DEC,2'b00,3'b001,3'b011,1'b1,1'b0,1'bx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
{`DEC,2'b00,3'b001,3'b011,1'b0,1'b1,1'bx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
{`DEC,2'b00,3'b001,3'b011,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
  //BLE instructions
{`DEC,2'b00,3'b001,3'b100,1'b1,1'b0,1'bx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
{`DEC,2'b00,3'b001,3'b100,1'b0,1'b1,1'bx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
{`DEC,2'b00,3'b001,3'b100,2'bxx,1'b1}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
{`DEC,2'b00,3'b001,3'b100,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};

 //Lab 8 branch instructions for function calls
  //BL instruction
{`DEC,2'b11,3'b010,3'b111,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1, 2'b11,3'b010,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`ADDsx8};
  //BX instruction
{`DEC,2'b00,3'b010,3'b000,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`mov_B,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0, 2'b10,3'bxxx,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`NOTHING};
  //BLX instruction
{`DEC,2'b10,3'b010,3'b111,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`mov_B,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1, 2'b11,3'b010,1'b0,1'b1,1'b0,1'b0,1'b1,2'b10,`NOTHING};
	
	//States that move value of Rd onto datapath out and set PC = R7 for both blx and bx 
{`mov_B,2'b10,3'b010,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`ALU_AB,1'b0,1'b0,1'b1,1'b1,1'b0,1'b0,1'b0, 2'b10,3'b010,1'b0,1'b0,1'b0,1'b0,1'b0,2'b10,`NOTHING};
{`ALU_AB,2'b10,3'b010,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`set_PC,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0, 2'b10,3'b010,1'b0,1'b0,1'b0,1'b0,1'b0,2'b10,`NOTHING};
{`set_PC,2'b10,3'b010,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0, 2'b10,3'b010,1'b0,1'b1,1'b0,1'b0,1'b0,2'b10,`EQRD};

{`mov_B,2'b00,3'b010,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`ALU_AB,1'b1,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0, 2'b10,3'b010,1'b0,1'b0,1'b0,1'b0,1'b0,2'b10,`NOTHING};
{`ALU_AB,2'b00,3'b010,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`set_PC,1'b1,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0, 2'b10,3'b010,1'b0,1'b0,1'b0,1'b0,1'b0,2'b10,`NOTHING};
{`set_PC,2'b00,3'b010,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`IF1,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0, 2'b10,3'b010,1'b0,1'b1,1'b0,1'b0,1'b0,2'b10,`EQRD};

	//States for STR
{`mov_A,2'b00,3'b100,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`ALU_AB,1'b0,1'b1,1'b1,1'b0,1'b0,1'b0,1'b0, 2'b11,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'b1,2'b10,`NOTHING};
{`ALU_AB,2'b00,3'b100,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`STR_Addr,1'b0,1'b1,1'b1,1'b0,1'b1,1'b1,1'b0,2'b11,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'b1,2'b10,`NOTHING};
{`STR_Addr,2'b00,3'b100,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`MOV_im8,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,2'b10,3'bxxx,1'b0,1'b0,1'b0,1'b1,1'b1,2'b10,`NOTHING};
       //State name because ALU_AB would cause it to get stuck in a loop
{`MOV_im8,2'b00,3'b100,3'bxxx,3'bxxx}:  {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`STR_value,1'b1,1'b0,1'bx,1'b1,1'b1,1'b1,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
{`STR_value,2'b00,3'b100,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`STR_value2,1'bx,1'bx,1'bx,1'bx,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b0,`MWRITE,`NOTHING};
{`STR_value2,2'b00,3'b100,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`IF1,1'bx,1'bx,1'bx,1'bx,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b0,2'b10,`NOTHING};

        //States for LDR
{`mov_A,2'b00,3'b011,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`ALU_AB,1'b0,1'b1,1'b1,1'b0,1'b0,1'b0,1'b0, 2'b11,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b1,2'b10,`NOTHING};
{`ALU_AB,2'b00,3'b011,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`STR_Addr,1'b0,1'b1,1'b1,1'b0,1'b1,1'b1,1'b0,2'b11,3'bxxx,1'b0,1'b0,1'b0,1'b1,1'b1,2'b10,`NOTHING};
{`STR_Addr,2'b00,3'b011,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`STR_value,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,2'b10,3'bxxx,1'b0,1'b0,1'b0,1'b1,1'b1,2'b10,`NOTHING};
{`STR_value,2'b00,3'b011,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`STR_value2,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b0,2'b10,`NOTHING};
{`STR_value2,2'b00,3'b011,3'bxxx,3'bxxx}: {next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`WRITE_REG,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'b0,1'b0,2'b10,`NOTHING};
{`WRITE_REG,2'b00,3'b011,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`IF1,1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b1,2'b10,3'b100,1'b0,1'b0,1'b0,1'b0,1'b0,`MREAD,`NOTHING};
	//Halt state        
{`DEC,2'b00,3'b111,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`HALT,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'bx,1'b0,1'b1,1'bx,1'bx,2'b10,`NOTHING};
{`HALT,2'bxx,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`HALT,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'bxx,3'bxxx,1'bx,1'b0,1'b1,1'bx,1'bx,2'b10,`NOTHING};

	 //State for MOV Rn,#<im8>
{`MOV_im8,2'bxx,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8}= {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,2'b11,3'b011,1'bx,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
         
	 //States for MOV Rd,Rm{,<sh_op>}      
{`MOV_rmb,2'bxx,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`SHIFT_rmb,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,2'b01,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
{`SHIFT_rmb,2'bxx,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`WRITE_REG,1'b1,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
	 
         //States for ADD Rd,Rn,Rm{,<sh_op>} and mov to and and b for all alu
{`mov_A,2'bxx,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`mov_B,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,2'b11,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
{`mov_B,2'bxx,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} =  {`ALU_AB,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,2'b01,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
	 //State for Add
{`ALU_AB,2'b00,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = { `WRITE_REG,1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
	 //State for CMP Rn,Rm{,<sh_op>}
{`ALU_AB,2'b01,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`IF1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
         //State for AND Rd,Rn,Rm{,sh_op}
{`ALU_AB,2'b10,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`WRITE_REG,1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
         
	 //State for MVN Rd,Rm{,<sh_op>}
{`mov_nB,2'b11,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`ALU_AB,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,2'b01,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
{`ALU_AB,2'b11,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`WRITE_REG,1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,2'bxx,3'bxxx,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};


         //State for writing into register Rd
{`WRITE_REG,2'bxx,3'bxxx,3'bxxx,3'bxxx}:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8} = {`IF1,1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b1,2'b10,3'b001,1'b0,1'b0,1'b0,1'bx,1'bx,2'b10,`NOTHING};
	 

default:{next_state,asel,bsel,loada,loadb,loadc,loads,write,nsel,vsel,load_ir,load_pc,reset_pc,load_addr,addr_sel,mem_cmd,add_sximm8}={4'bxxxx,1'bx,1'bx,1'bx,1'bx,1'bx,1'bx,1'bx,2'bxx,3'bxxx,1'bx,1'bx,1'bx,1'bx,1'bx,2'bxx,1'bx,`NOTHING};
	endcase
	end
endmodule


//Module for Instruction decoder
module instruct_dec(in,shift,sximm5,sximm8,ALUop,opcode,op,nsel,readnum,writenum,cond);
input [15:0] in;
input [1:0] nsel;
output [15:0] sximm5,sximm8;
output [1:0] ALUop, op, shift;
output [2:0] opcode,readnum,writenum,cond;

wire [15:0] sximm5,sximm8;
wire [1:0] ALUop, op, shift;
wire [2:0] opcode,mux_out,cond;

//Assigning bits of in to outputs which will go to fsm and datapath
assign sximm5 = {{11{in[4]}},in[4:0]};
assign sximm8 = {{8{in[7]}},in[7:0]};
assign ALUop = in[12:11];
assign op = in[12:11];
assign shift = in[4:3];
assign opcode = in[15:13];
assign cond = in[10:8];
//input 1 is Rn, input 2 is Rd, input 3 is Rm
Mux3a mux(in[10:8],in[7:5],in[2:0],nsel,mux_out);

assign readnum = mux_out;
assign writenum  = mux_out;

endmodule

//Using Mux style from slide set, except 4 inputs 3 bit inputs and outputs
module Mux3a(a2,a1,a0,sel,out);
 input[2:0] a2,a1,a0;
 input[1:0] sel;
 output[2:0] out;
 reg[2:0] out;
 
 //Checking which register was loaded
 always @(*)begin
   case(sel)
    2'b01: out = a0;
    2'b10: out = a1;
    2'b11: out = a2;
    default: out = {3{1'bx}};
   endcase
 end
endmodule

//Load enable register size n
module LEvdFF(in,load,clk,out);
parameter n= 8;
input[n-1:0] in;
input load, clk;
output[n-1:0] out;
wire[n-1:0] flip_in, out;
//Instantiating flip flop
vDFF#(n) ff(clk,flip_in,out);
//Load in mux
assign flip_in = load ? in : out;
endmodule

//Sets value of pc based on whether a branch instruction was used
module pc_adder(next_pc,sximm8,add_sximm8,PC_out,datapath_out);
 input[8:0] next_pc,sximm8,datapath_out;
 input[1:0] add_sximm8;
 output[8:0] PC_out;
 reg[8:0] PC_out;

`define ADDsx8 2'b01
`define NOTHING 2'b00
`define EQRD 2'b11
 
 always @(*)begin
	case(add_sximm8)
	2'b00: PC_out = next_pc;

        2'b01: if(sximm8[8])begin 
	      PC_out = (next_pc -(~sximm8 + 1'b1)) -1'b1; 
		   end
	       else begin
               PC_out = next_pc + sximm8 -1'b1;                
		   end
	
	2'b11: PC_out = datapath_out;

	default: PC_out = {9{1'bx}};
	endcase
 end
endmodule
