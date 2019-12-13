`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
				output        dc_en,
            input  [31:0] instr,
				output        pcsrc,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg, branch, branch_NE;
  wire         zero;
  wire        alusrc, regdst, regwrite, jump, jreg, jlink;
  wire [2:0]  alucontrol;
  wire       memwriteff3, regwriteff;
  wire [31:0] afterinstr; // branch instruction
  wire		  jumpfff, jregfff, jlinkfff;
  
  
  // branchmux
  mux2 #(32) branchmux(
    .d0  (instr),
    .d1  (32'b0),
    .s   (pcsrc == 1'b1 ||jumpfff == 1'b1 ||jregfff == 1'b1 ||jlinkfff == 1'b1),
    .y   (afterinstr));
	 
	 // jump flipflop //
  flopr #(1) juff(
		.d   (jump), 
		.reset (reset),
		.clk  (clk), 
		.q    (jumpfff));
	// jump flipflop //
	
	// jreg flipflop1 //
  flopr #(1) jrff(
		.d   (jreg), 
		.reset (reset),
		.clk  (clk), 
		.q    (jregfff));
	// jreg flipflop1 //
	
	// jlink flipflop1 //
  flopr #(1) brnchff(
		.d   (jlink), 
		.reset (reset),
		.clk  (clk), 
		.q    (jlinkfff));
	// jlink flipflop1 //
  
  // Instantiate Controller
  controller c(
		.reset      (reset),
		.clk        (clk),
		.op         (afterinstr[31:26]), 
      .funct      (afterinstr[5:0]), 
      .zero       (zero),
      .signext    (signext),
      .shiftl16   (shiftl16),
      .memtoreg   (memtoreg),
      .memwrite   (memwriteff3),
      .pcsrc      (pcsrc),
      .alusrc     (alusrc),
      .regdst     (regdst),
      .regwrite   (regwriteff),
      .jump       (jump),
		.jreg       (jreg),
		.jlink      (jlink),
      .alucontrol (alucontrol));
	

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
	 .memwriteff   (memwriteff),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
	 .jreg       (jreg),
	 .jlink      (jlink),
    .alucontrol (alucontrol),
	 .memwrite   (memwrite),
    .zero       (zero),
    .pc         (pc),
    .instr      (afterinstr),
    .aluout     (memaddr),
	 .dc_en      (dc_en), 
	 .control_en (control_en),
    .writedata3  (memwritedata),
    .readdata   (memreaddata));
	 
	 	// mux for hazard detection unit control_en
  mux2 #(1) control_enff1(
    .d0  (1'b0),
    .d1  (memwriteff3),
    .s   (control_en),
    .y   (memwriteff));
	 
	 // mux for hazard detection unit control_en
  mux2 #(1) control_enff2(
    .d0  (1'b0),
    .d1  (regwriteff),
    .s   (control_en),
    .y   (regwrite));
	 
	 

endmodule


							 

module controller(input         clk, reset,
						input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump,jreg, jlink,
                  output [2:0] alucontrol);

  wire [1:0] aluop, aluopff;
  wire       branch, branchff, branchff2, branch_NE, branch_NEff, branch_NEff2;
  wire       check, check1, check2;
  wire [5:0] opff, functff;

  maindec md(
    .op       (op),
	 .funct    (funct),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .branch_NE(branch_NE),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
	 .jreg     (jreg),
	 .jlink    (jlink),
    .aluop    (aluop));

  aludec ad( 
    .op         (opff),
    .funct      (functff),
    .aluop      (aluopff), 
    .alucontrol (alucontrol));
	   
// branch flipflop1 //
  flopr #(6) off(
		.d   (op), 
		.reset (reset),
		.clk  (clk), 
		.q    (opff));
	// branch flipflop1 //
	
	  
// branch flipflop1 //
  flopr #(6) funcff(
		.d   (funct), 
		.reset (reset),
		.clk  (clk), 
		.q    (functff));
	// branch flipflop1 //
	 
	  
// branch flipflop1 //
  flopr #(1) brnchff(
		.d   (branch), 
		.reset (reset),
		.clk  (clk), 
		.q    (branchff));
	// branch flipflop1 //
	  
// branch flipflop2 //
  /*flopr #(1) brnchff2(
		.d   (branchff), 
		.reset (reset),
		.clk  (clk), 
		.q    (branchff2));*/
	// branch flipflop2 //
	
	  
// notbranch flipflop1 //
  flopr #(1) nbrnchff(
		.d   (branch_NE), 
		.reset (reset),
		.clk  (clk), 
		.q    (branch_NEff));
	// notbranch flipflop1 //
	
	  
// notbranch flipflop2 //
  /*flopr #(1) nbrnchff2(
		.d   (branch_NEff), 
		.reset (reset),
		.clk  (clk), 
		.q    (branch_NEff2));*/
	// notbranch flipflop2 //
	
	// aluop flipflop //
  flopr #(2) alopff(
		.d   (aluop), 
		.reset (reset),
		.clk  (clk), 
		.q    (aluopff));
	// aluop flipflop //
	
	
  assign pcsrc = (branchff && zero) || (branch_NEff && ~zero);

endmodule


module maindec(input  [5:0] op, 
					input  [5:0] funct, // opcode implementation
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, branch_NE, alusrc,
               output       regdst, regwrite,
               output       jump, jreg, jlink,
               output [1:0] aluop);

  reg [13:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, branch_NE, memwrite,
          memtoreg, jump, jreg, jlink, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: case(funct)
						6'b001000:	controls <= #`mydelay 14'b00110000001011; // Rtype
						default:    controls <= #`mydelay 14'b00110000000011; // Rtype
					  endcase
      6'b100011: controls <= #`mydelay 14'b10101000100000; // LW
      6'b101011: controls <= #`mydelay 14'b10001001000000; // SW
      6'b000100: controls <= #`mydelay 14'b10000100000001; // BEQ
      6'b000101: controls <= #`mydelay 14'b10000010000001; // BNE
		6'b001011, 
		6'b001010: controls <= #`mydelay 14'b10101000000000; // SLTI
      6'b001000, 
      6'b001001: controls <= #`mydelay 14'b10101000000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 14'b00101000000010; // ORI
      6'b001111: controls <= #`mydelay 14'b01101000000000; // LUI
		6'b000011: controls <= #`mydelay 14'b00100000010100; // JAL
      6'b000010: controls <= #`mydelay 14'b00000000010000; // J
      default:   controls <= #`mydelay 14'bxxxxxxxxxxxxxx; // ???
    endcase

endmodule


module aludec(input      [5:0] op,
				  input      [5:0] funct, // func instruction implementation
              input      [1:0] aluop,
              output reg [2:0] alucontrol);

  always @(*)
  case(op)
		6'b001011,
		6'b001010: alucontrol<= #`mydelay 3'b111; // SLTI
    default: case(aluop)
      2'b00: alucontrol <= #`mydelay 3'b010;  // add
      2'b01: alucontrol <= #`mydelay 3'b110;  // sub
      2'b10: alucontrol <= #`mydelay 3'b001;  // or
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 3'b010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 3'b110; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 3'b000; // AND
          6'b100101: alucontrol <= #`mydelay 3'b001; // OR
          6'b101011,
          6'b101010: alucontrol <= #`mydelay 3'b111; // SLT, SLTU
          default:   alucontrol <= #`mydelay 3'bxxx; // ???
        endcase
    endcase
	endcase
    
endmodule

// hazard detection unit //

module hazarddetectionunit(input [4:0] dc_rs, dc_rt, // decoding stage rs, rt
									input [4:0] ex_rt, //execution stage rt, which is destination for lw, sw instructions
									//input  ex_memwrite,
									input  ex_memtoreg,
									output reg pcwrite_en,
									output reg dc_en,
									output reg control_en);
		wire [4:0] check_rs, check_rt;
		
		assign check_rs = dc_rs ^ ex_rt;
		assign check_rt = dc_rt ^ ex_rt;
	always@(*)
		begin
		if((check_rs == 5'b00000 || check_rt == 5'b00000) && (ex_memtoreg == 1'b1 ))
			begin
				pcwrite_en <= #`mydelay 1'b0;
				dc_en <= #`mydelay 1'b0;
				control_en <= #`mydelay 1'b0;
			end
		else
			begin
				pcwrite_en <= #`mydelay 1'b1;
				dc_en <= #`mydelay 1'b1;
				control_en <= #`mydelay 1'b1;
			end
			end


endmodule

module forwardingunit(input [4:0] ex_rs, ex_rt,
                      input [4:0] dc_rs, dc_rt,
							 input [4:0] mem_rd, wb_rd,
							 input mem_regwrite, wb_regwrite,
							 input regdst, memtoreg, pcsrc,
							 output reg dc_srca2, dc_writedata, 
							 output reg [1:0] ex_srca, ex_srcb);
	wire [4:0] check_memex1, check_wbex1, check_wbdc1, check_wbdc2;
	
	wire  [4:0] check_memex2, check_wbex2;
	assign check_memex1 = ex_rs ^ mem_rd;
   assign check_wbex1 = ex_rs ^ wb_rd;
   assign check_wbdc1 = dc_rs ^ wb_rd;
   assign check_wbdc2 = dc_rt ^ wb_rd;	
	assign check_memex2 = (ex_rt ^ mem_rd);
	assign check_wbex2  = ex_rt ^ wb_rd;
	
	
	/*always@(*)
		begin
			if(regdst == 1'b1 || memtoreg == 1'b1 ||pcsrc == 1'b1 ) check_memex2 <= (ex_rt ^ mem_rd);
			else  check_memex2 <= 5'b11111;
			
			if(regdst == 1'b1 || memtoreg == 1'b1 || pcsrc == 1'b1) check_wbex2 <= ex_rt ^ wb_rd;
			else  check_wbex2 <= 5'b11111;
		end
		*/
	
	
	
	always@(*)
		begin
			if(check_memex1 == 5'b00000 && mem_regwrite == 1'b1 && (mem_rd != 5'b00000) ) ex_srca <=  2'b10;  //forwarding mem to ex for srca`
			else if(check_wbex1 == 5'b00000 && wb_regwrite == 1'b1 && (wb_rd != 5'b00000) ) ex_srca <= 2'b01; //forwarding wb to ex for srca 
			else ex_srca <=  2'b00;
			
			if(check_memex2 == 5'b00000 && mem_regwrite == 1'b1 && (mem_rd != 5'b00000) ) 
				begin
					if(regdst == 1'b0 || memtoreg == 1'b0) ex_srcb <=  2'b00;
						if(pcsrc) ex_srcb <=  2'b10;
						else ex_srcb <=  2'b10;
				end
					else if(wb_regwrite == 1'b1 && (wb_rd != 5'b00000) && (check_wbex2 == 5'b00000))
				begin	
						if(~regdst && ~memtoreg) ex_srcb <=  2'b00;
						if(pcsrc) ex_srcb <=  2'b10;
						else ex_srcb <=  2'b01;
				end
					 //forwarding mem to ex for srcb 
			else if(check_wbex2 == 5'b00000 && wb_regwrite ==1'b1 && (wb_rd != 5'b00000)) ex_srcb <=  2'b01; //forwarding wb to ex for srcb
			else ex_srcb <=  2'b00;
			
			if(check_wbdc1 == 5'b00000  && wb_regwrite == 1'b1&& (wb_rd != 5'b00000) ) dc_srca2 <=  1'b1; // forwarding wb to dc for srca2
			else dc_srca2 <=  1'b0;
			
			if(check_wbdc2 == 5'b00000 && wb_regwrite == 1'b1&& (wb_rd != 5'b00000)) dc_writedata <= 1'b1; //forwading wb to dc for writedata, if we change him we get 11000000
			else dc_writedata <= 1'b0;		
		
		end
	
endmodule
// end forwarding unit module //

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst, memwriteff,
                input         regwrite, jump, jreg, jlink,
                input  [2:0]  alucontrol,
					 output        memwrite,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata3,
					 output        dc_en, control_en,
                input  [31:0] readdata);
  wire        jumpff, jumpff2, jumpff3, jregff, jregff2, jregff3, jlinkff, jlinkff2, jlinkff3;
  wire [31:0] writedata, writedata2, forwardwritedata;
  wire [4:0]  writereg, writereg2, writereg3, writejreg;
  wire [31:0] pcnext, pcnextbr, pcnextjreg, pcplus4, pcplus42, pcplus43, pcbranch, pcbranch2;
  wire [31:0] signimm, signimmsh, shiftedimm, beforesignimm;
  wire [31:0] srca, srcb, srca2, forwardsrca2, forwardsrcb, forwardsrca;
  wire [31:0] result, jresult;
  wire        shift;
  wire [4:0]  regrs, regrt; //rs, rt register
  wire [4:0]  regdst1, regdst2;
  wire [31:0] aluout2;
  wire [31:0] aluout3;
  wire [31:0] readdata2;
  wire        zero2;
  wire        shiftl16ff, regdstff, alusrcff, memtoregff, memtoregff2, memtoregff3, regwriteff, regwriteff2, regwriteff3; // control flipflop
  wire        dcsrca2, dcwritedata; // forwarding unit control
  wire [1:0]  exsrca, exsrcb; // forwarding unit control
  wire        pcwrite_en; // hazard detection unit control
  wire        memwriteff2;
  wire [31:0] instrff;

  // next PC logic // hazard detection unit
  flopenr #(32) pcreg(
    .clk   (clk),
	 .en    (pcwrite_en),
    .reset (reset),
    .d     (pcnext),
    .q     (pc));
  
  // flipflop for instruction //
	flopr #(32) instrff1(
		.d   (instr), 
		.reset (reset),
		.clk  (clk), 
		.q    (instrff));
  
  // PC+4 logic
  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus42));
	 
	// flipflop for Pc+4 adder //
	flopr #(32) pcadderff1(
		.d   (pcplus42), 
		.reset (reset),
		.clk  (clk), 
		.q    (pcplus43));
		
	flopr #(32) pcadderff2(
		.d   (pcplus43), 
		.reset (reset),
		.clk  (clk), 
		.q    (pcplus4));
	// flipflop for Pc+4 adder //
	 
	 
 //sign immediate logic
  sl2 immsh(
    .a (signimm),
    .y (signimmsh));
  //         
  adder pcadd2(
    .a (pcplus43),
    .b (signimmsh),
    .y (pcbranch2));
	 
// pcbranch flipflop //
  flopr #(32) pcbranchff(
		.d   (pcbranch2), 
		.reset (reset),
		.clk  (clk), 
		.q    (pcbranch));
	// pcbranch flipflop //
	
	// made change for do because of pcadderff1
  mux2 #(32) pcbrmux(
    .d0  (pcplus42),
    .d1  (pcbranch2),
    .s   (pcsrc),
    .y   (pcnextbr));
	 
	 //made for jr instruction
  mux2 #(32) pcjmux( 
	 .d0  (pcnextbr),
	 .d1  (srca),
	 .s   (jregff),
	 .y   (pcnextjreg));
	 
	 //made for jump instruction 
  mux2 #(32) pcmux(
    .d0   (pcnextjreg),
    .d1   ({pcplus4[31:28], instrff[25:0], 2'b00}),
    .s    (jumpff),
    .y    (pcnext));
	 
	 // regwrite flipflop1 //
  flopr #(1) rfwriteff(
		.d   (regwrite), 
		.reset (reset),
		.clk  (clk), 
		.q    (regwriteff));
	// regwrite flipflop1 //
	
	 // regwrite flipflop2 //
  flopr #(1) rfwriteff2(
		.d   (regwriteff), 
		.reset (reset),
		.clk  (clk), 
		.q    (regwriteff2));
	// regwrite flipflop2 //
	
	 // regwrite flipflop3 //
  flopr #(1) rfwriteff3(
		.d   (regwriteff2), 
		.reset (reset),
		.clk  (clk), 
		.q    (regwriteff3));
	// regwrite flipflop3 //
	 
	 
	 // jump flipflop1 //
  flopr #(1) jupff(
		.d   (jump), 
		.reset (reset),
		.clk  (clk), 
		.q    (jumpff));
	// jump flipflop1 //
	
	 // jump flipflop2 //
  flopr #(1) jupff2(
		.d   (jumpff), 
		.reset (reset),
		.clk  (clk), 
		.q    (jumpff2));
	// jump flipflop2 //
	
	 // jump flipflop3 //
  flopr #(1) jupff3(
		.d   (jumpff2), 
		.reset (reset),
		.clk  (clk), 
		.q    (jumpff3));
	// jump flipflop3 //
	 
 // jreg flipflop1 //
  flopr #(1) jreff(
		.d   (jreg), 
		.reset (reset),
		.clk  (clk), 
		.q    (jregff));
	// jreg flipflop1 //
	
	 // jreg flipflop2 //
  flopr #(1) jreff2(
		.d   (jregff), 
		.reset (reset),
		.clk  (clk), 
		.q    (jregff2));
	// jreg flipflop2 //
	
	 // jreg flipflop3 //
  flopr #(1) jreff3(
		.d   (jregff2), 
		.reset (reset),
		.clk  (clk), 
		.q    (jregff3));
	// jreg flipflop3 //
	 
	  // jlink flipflop1 //
  flopr #(1) jlinff(
		.d   (jlink), 
		.reset (reset),
		.clk  (clk), 
		.q    (jlinkff));
	// jlink flipflop1 //
	
	 // jlink flipflop2 //
  flopr #(1) jlinff2(
		.d   (jlinkff), 
		.reset (reset),
		.clk  (clk), 
		.q    (jlinkff2));
	// jlink flipflop2 //
	
	 // jlink flipflop3 //
  flopr #(1) jlinff3(
		.d   (jlinkff2), 
		.reset (reset),
		.clk  (clk), 
		.q    (jlinkff3));
	// jlink flipflop3 //
	 
	 
  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwriteff3),
    .ra1     (instr[25:21]), // rs
    .ra2     (instr[20:16]), // rt
    .wa      (writejreg),
    .wd      (jresult),
    .rd1     (forwardsrca2),
    .rd2     (forwardwritedata));
	 
	 // register address1 flipflop //
  flopr #(5) regadd1ff(
		.d   (instr[25:21]), 
		.reset (reset),
		.clk  (clk), 
		.q    (regrs));
	// register address1 flipflop //
	 
	 // register address2 flipflop //
  flopr #(5) regadd2ff(
		.d   (instr[20:16]), 
		.reset (reset),
		.clk  (clk), 
		.q    (regrt));
	// register address2 flipflop //
	 
	 
	 
	 
   // register destination1 flipflop //
  flopr #(5) regdest1ff(
		.d   (instr[20:16]), 
		.reset (reset),
		.clk  (clk), 
		.q    (regdst1));
	// register destination1 flipflop //
	 
   // register destination2 flipflop //
  flopr #(5) regdest2ff(
		.d   (instr[15:11]), 
		.reset (reset),
		.clk  (clk), 
		.q    (regdst2));
	// register destination2 flipflop //
	
	   // register destination control flipflop //
  flopr #(1) regdestconftrolff(
		.d   (regdst), 
		.reset (reset),
		.clk  (clk), 
		.q    (regdstff));
	// register destination control flipflop //
	
	
  mux2 #(5) wrmux(
    .d0  (regdst1),
    .d1  (regdst2),
    .s   (regdstff),
    .y   (writereg2));

// writereg1 flipflop //
  flopr #(5) wreg1ff(
		.d   (writereg2), 
		.reset (reset),
		.clk  (clk), 
		.q    (writereg3));
	// writereg1 flipflop //
	
	// writereg2 flipflop //
  flopr #(5) wreg2ff(
		.d   (writereg3), 
		.reset (reset),
		.clk  (clk), 
		.q    (writereg));
	// writereg2 flipflop //
	 
	 
	 
	 
	 //jal mux
  mux2 #(5) wjalmux(
	 .d0  (writereg),
	 .d1  (5'b11111),
	 .s   (jlinkff3),
	 .y   (writejreg));
	 
	 
   // read data flipflop //
	flopr #(32) rdff(
		.d   (readdata), 
		.reset (reset),
		.clk  (clk), 
		.q    (readdata2));
	// read data flipflop //  
	
	// memtoreg flipflop //
	flopr #(1) mmtoregff(
		.d   (memtoreg), 
		.reset (reset),
		.clk  (clk), 
		.q    (memtoregff));
	// memtoreg flipflop //
	
	// memtoreg flipflop //
	flopr #(1) mmtoregff2(
		.d   (memtoregff), 
		.reset (reset),
		.clk  (clk), 
		.q    (memtoregff2));
	// memtoreg flipflop // 
	
	// memtoreg flipflop //
	flopr #(1) mmtoregff3(
		.d   (memtoregff2), 
		.reset (reset),
		.clk  (clk), 
		.q    (memtoregff3));
	// memtoreg flipflop //  
	
	
 // mux that has memtoreg //
  mux2 #(32) resmux(
    .d0 (aluout3),
    .d1 (readdata2),
    .s  (memtoregff3),
    .y  (result));
	 
	 
	 //jal mux
  mux2 #(32) resjmux(
    .d0 (result),
	 .d1 (pcplus4),
	 .s  (jlinkff3),
	 .y  (jresult));

  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (signext),
    .y       (beforesignimm[31:0]));
	 
	// flipflop for sign extend //
	flopr #(32) sigextendff(
		.d   (beforesignimm[31:0]), 
		.reset (reset),
		.clk  (clk), 
		.q    (signimm[31:0]));
	// flipflop for sign extend // 
   
	// shiftl16 flipflop //
	flopr #(1) shifl16ff(
		.d   (shiftl16), 
		.reset (reset),
		.clk  (clk), 
		.q    (shiftl16ff));
	// flipflop for sign extend // 
   
	
  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16ff),
    .y         (shiftedimm[31:0]));

	 // read data2 flipflop //
  flopr #(32) rd2ff(
		.d   (writedata), 
		.reset (reset),
		.clk  (clk), 
		.q    (writedata2));
	// read data2 flipflop //
	
	// writedata flipflop //
  flopr #(32) wdff(
		.d   (forwardsrcb), 
		.reset (reset),
		.clk  (clk), 
		.q    (writedata3));
	// writedata flipflop //
	 
	 
	 // alusrc flipflop //
  flopr #(1) aluscff(
		.d   (alusrc), 
		.reset (reset),
		.clk  (clk), 
		.q    (alusrcff));
	// alusrc flipflop //
	 
  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (forwardsrcb),
    .d1 (shiftedimm[31:0]),
    .s  (alusrcff),
    .y  (srcb));
	 
	 
	 // forwarding unit srca
  mux3 #(32) forwardsrcamux(
    .d0 (forwardsrca),
	 .d1 (result),
	 .d2 (aluout),
	 .s  (exsrca),
	 .y  (srca));
	 
	 
	 // forwarding unit srcb
  mux3 #(32) forwardsrcbmux(
    .d0 (writedata2),
	 .d1 (result),
	 .d2 (aluout),
	 .s  (exsrcb),
	 .y  (forwardsrcb));
	 
	 
	 
	 
	 
	// forwarding unit srca2
  mux2 #(32) forwardsrca2mux(
    .d0 (forwardsrca2),
    .d1 (result),
    .s  (dcsrca2),
    .y  (srca2));
		 
	 
	 
	 
	 	// forwarding unit writedata
  mux2 #(32) forwardwritedatamux(
    .d0 (forwardwritedata),
    .d1 (result),
    .s  (dcwritedata),
    .y  (writedata));
		 
  // read data1 flipflop //
  flopr #(32) rd1ff(
		.d   (srca2), 
		.reset (reset),
		.clk  (clk), 
		.q    (forwardsrca));
	// read data1 flipflop //
	
	
	// alucontrol flipflop //
 
  //flopr #(3) alcontrolff(
	//	.d   (alucontrol), 
	//	.reset (reset),
	//	.clk  (clk), 
	//	.q    (alucontrolff));
	// alucontrol flipflop //
	
	 
	 
  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout2),
    .zero    (zero));
	 
	  
	// zero flipflop //
	/*flopr #(1) zeroff(
		.d   (zero2), 
		.reset (reset),
		.clk  (clk), 
		.q    (zero));*/
	// zero flipflop //  
	
	 
	// aluresult flipflop //
	flopr #(32) aluout1ff(
		.d   (aluout2), 
		.reset (reset),
		.clk  (clk), 
		.q    (aluout));
	// aluresult flipflop //  
	
	 
	// aluresult flipflop2 //
	flopr #(32) aluout2ff(
		.d   (aluout), 
		.reset (reset),
		.clk  (clk), 
		.q    (aluout3));
	// aluresult flipflop2 //  
	
	
	// memwrite flipflop1 //
  flopr #(1) mmwriteff(
		.d   (memwriteff), 
		.reset (reset),
		.clk  (clk), 
		.q    (memwriteff2));
	// memwrite flipflop1 //
	
	// memwrite flipflop2 //
  flopr #(1) mmwriteff2(
		.d   (memwriteff2), 
		.reset (reset),
		.clk  (clk), 
		.q    (memwrite));
	// memwrite flipflop2 //
	
	

	// call forwarding module //
	forwardingunit forward(
		.ex_rs(regrs),
		.ex_rt(regrt),
		.dc_rs(instr[25:21]),
		.dc_rt(instr[20:16]),
		.mem_rd(writereg3),
		.wb_rd(writejreg),
		.mem_regwrite(regwriteff2),
		.wb_regwrite(regwriteff3),
		.regdst     (regdstff),
		.memtoreg   (memtoregff3),
		.pcsrc      (pcsrc),
		.dc_srca2(dcsrca2),
		.dc_writedata(dcwritedata),
		.ex_srca(exsrca),
		.ex_srcb(exsrcb));
	// call forwarding module //
	
	
	hazarddetectionunit hazard(
		.dc_rs(instr[25:21]),
		.dc_rt(instr[20:16]),
		.ex_rt(regdst1),
		//.ex_memwrite(memwriteff2),
		.ex_memtoreg(memtoregff),
		.pcwrite_en(pcwrite_en),
		.dc_en(dc_en),
		.control_en(control_en));
	 
  
    
endmodule
