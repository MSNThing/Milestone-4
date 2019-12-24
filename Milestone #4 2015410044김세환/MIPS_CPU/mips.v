`timescale 1ns/1ps
`define mydelay 0.2

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] inst,
            output        MAmemwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg;
  wire [31:0] instr, pcplus4, IDpcplus4;
  wire [1:0]  branch;
  wire        alusrc, regdst, regwrite, jump, memwrite, stall;
  wire [3:0]  alucontrol;
//Instruction Fetch Stage//
//store instruction from memory and pcplus4 from pclogic
  IF #(32) IFtoID(
    .inst    (inst[31:0]),
    .pcplus4 (pcplus4),
    .clk    (clk),
    .reset  (reset),
    .stall    (stall),
    .instr    (instr[31:0]),
    .IDpcplus4    (IDpcplus4));

//In ID stage, instruction from IF stage will be decoded by controller c
//then controller send those decoded information and control units to datapath dp
  // Instantiate Controller
  controller c(
      .op         (instr[31:26]), 
		.funct      (instr[5:0]), 
		.signext    (signext),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (memwrite),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
      .branch     (branch),
		.alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .memwrite    (memwrite),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .branch      (branch),
    .alucontrol (alucontrol),
    .MAmemwrite    (MAmemwrite),
    .stall       (stall),
    .pc         (pc),
    .instr      (instr),
    .MAaluout     (memaddr), 
    .MAwritedata  (memwritedata),
    .pcplus4    (pcplus4),
    .IDpcplus4    (IDpcplus4),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       alusrc,
                  output       regdst, regwrite,
                  output       jump,
                  output [1:0] branch,
                  output [3:0] alucontrol);

  wire [1:0] aluop;

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .aluop    (aluop));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol));

endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output [1:0] branch,
               output       alusrc, regdst, regwrite,
               output       jump,
               output [1:0] aluop);

  reg [11:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 12'b001100000011; // Rtype
      6'b100011: controls <= #`mydelay 12'b101010001000; // LW
      6'b101011: controls <= #`mydelay 12'b100010010000; // SW
      6'b000100: controls <= #`mydelay 12'b100000100001; // BEQ
      6'b000101: controls <= #`mydelay 12'b100001100001; // BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 12'b101010000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 12'b001010000010; // ORI
      6'b001111: controls <= #`mydelay 12'b011010000000; // LUI
      6'b000010: controls <= #`mydelay 12'b000000000100; // J
      6'b000011: controls <= #`mydelay 12'b001000000100; // JAL
      default:   controls <= #`mydelay 12'bxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg [3:0] alucontrol);

  always @(*)
    case(aluop)
      2'b00: alucontrol <= #`mydelay 4'b0010;  // add
      2'b01: alucontrol <= #`mydelay 4'b1010;  // sub
      2'b10: alucontrol <= #`mydelay 4'b0001;  // or
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 4'b0010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 4'b1010; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
          6'b100101: alucontrol <= #`mydelay 4'b0001; // OR
          6'b101010: alucontrol <= #`mydelay 4'b1011; // SLT
          6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU
          6'b001000: alucontrol <= #`mydelay 4'b0100; // JR
          default:   alucontrol <= #`mydelay 4'bxxxx; // ???
        endcase
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, memwrite,
                input         alusrc, regdst,
                input         regwrite, jump,
                input  [1:0]  branch,
                input  [3:0]  alucontrol,
                output        MAmemwrite, stall,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] MAaluout, MAwritedata, pcplus4,
                input  [31:0] IDpcplus4, readdata);
//notation ID, EX, MA, WB of each wires and regs represent elements which are used in each stage.
//MA means MEM stage.
  wire [4:0]  WBwritereg, writerg, IDwritereg, EXwritereg, MAwritereg;
  wire [31:0] pcnext, pcnextbr, EXpcbranch, pcnextj;
  wire [31:0] EXpcplus4, EXinstr;
  wire [31:0] MAinstr, MApcplus4, MApcbranch, MAsrca;
  wire [31:0] WBinstr, WBpcplus4, WBaluout, WBreaddata;
  wire [31:0] EXsignimm, IDsignimm, EXsignimmsh, shiftedimm;
  wire [31:0] EXsrca, IDsrca, IDwritedata, IDsrcb, EXsrcb, EXwritedata, EXaluout;
  wire [31:0] tsrca, twritedata, WBresult, reslt;
  wire [3:0] EXalucontrol;
  wire        EXpcsrc, IDpcrtn, EXzero, EXpcrtn, MApcsrc, MApcrtn;
  wire        dh1, dh2, dh3, dh4, wt, wtime, _wtime, wclk;
  wire [9:0] IDcontrol;
  wire [7:0] EXcontrol;
  wire [3:0] MAcontrol;
  wire [2:0] WBcontrol;
//--------------------------------------------------------------------------------------
  assign EXpcsrc = EXcontrol[4] & (EXzero ^ EXcontrol[5]); //branch[0] and branch[1], pcsrc is calculated in EX stage
  assign IDpcrtn = (instr[31:26] == 6'b0 && instr[5:0] == 6'b001000) ? 1'b1 : 1'b0; //pcrtn is calculated in ID stage
  assign IDcontrol = {regdst, signext, shiftl16, alusrc, branch, memwrite, regwrite, memtoreg, jump}; //control[9:0]
 
//check whether data hazard occured or not in ID stage by checking control units and comparing register number of EX and MA stage.
  dhcheck dhchk(
    .instr (instr),
	 .EXwritereg (EXwritereg),
	 .MAwritereg (MAwritereg),
	 .regdst (regdst),
	 .memwrite (memwrite),
	 .EXregwrite (EXcontrol[2]), //regwrite
	 .MAregwrite (MAcontrol[2]),
	 .dh1 (dh1),
	 .dh2 (dh2),
	 .dh3 (dh3),
	 .dh4 (dh4));

//if data hazard occurs, check if it will be stalled or not
  dhcontrol dhcont(
    .dh1    (dh1),
    .dh2    (dh2),
    .dh3    (dh3),
    .dh4    (dh4),
    .EXmemtoreg    (EXcontrol[1]), //memtoreg
    .MAmemtoreg    (MAcontrol[1]),
    .stall    (stall));
//--------------------------------------------
  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .stall   (stall),
    .d     (pcnext),
    .q     (pc));


  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));
//caculate branch destination in EX stage
  sl2 immsh(
    .a (EXsignimm),
    .y (EXsignimmsh));
				 
  adder pcadd2(
    .a (EXpcplus4),
    .b (EXsignimmsh),
    .y (EXpcbranch));
//-------------------------------------
//branch implement in MA stage
  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (MApcbranch),
    .s   (MApcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcjmux(
    .d0   (pcnextbr),
    .d1   ({MApcplus4[31:28], MAinstr[25:0], 2'b00}),
    .s    (MAcontrol[0]), //jump
    .y    (pcnextj));

  mux2 #(32) pcmux(
    .d0   (pcnextj),
    .d1   (MAsrca),
    .s     (MApcrtn),
    .y     (pcnext));
//-------------------------------------
  // register file logic
//write register only in WB stage after calculate final data
  regfile rf(
    .clk     (clk),
	 .wt      (wt),
    .we      (WBcontrol[2]), //regwrite
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (WBwritereg),
    .wd      (WBresult),
    .rd1     (tsrca),
    .rd2     (twritedata));
//data fowarding whenever data hazards occur  
  dhfwd #(32) dhmux1(
    .d0    (tsrca),
    .EXdata    (EXaluout),
    .MAdata    (MAaluout),
    .EXsig    (dh1),
    .MAsig    (dh3),
    .stall   (stall),
    .y    (IDsrca));
  
  dhfwd #(32) dhmux2(
    .d0    (twritedata),
    .EXdata    (EXaluout),
    .MAdata    (MAaluout),
    .EXsig    (dh2),
    .MAsig    (dh4),
    .stall    (stall),
    .y    (IDwritedata));
//-----------------------------
  mux2 #(5) wrmux(
    .d0  (instr[20:16]),
    .d1  (instr[15:11]),
    .s   (IDcontrol[9]), //regdst
    .y   (writerg));
//get register number for writing 
  mux2 #(5) wrjmux(
    .d0 (writerg),
    .d1 (5'b11111),
    .s   (IDcontrol[0]), //jump
    .y   (IDwritereg));

  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (IDcontrol[8]), //signext
    .y       (IDsignimm[31:0]));

  shift_left_16 sl16(
    .a         (IDsignimm[31:0]),
    .shiftl16  (IDcontrol[7]), //shiftl16
    .y         (shiftedimm[31:0]));

  //get srcb for alu logic
  mux2 #(32) srcbmux(
    .d0 (IDwritedata),
    .d1 (shiftedimm[31:0]),
    .s  (IDcontrol[6]), //alusrc
    .y  (IDsrcb));
//load register between ID stage and EX stage	 
  ID #(32) IDtoEX(
    .IDinstr    (instr),
    .IDsrca    (IDsrca),
	 .IDsrcb    (IDsrcb),
    .IDwritedata    (IDwritedata),
    .IDsignimm    (IDsignimm),
    .IDpcplus4    (IDpcplus4),
    .IDwritereg    (IDwritereg),
    .alucontrol    (alucontrol),
    .IDcontrol    (IDcontrol),
    .IDpcrtn    (IDpcrtn),
    .clk    (clk),
    .reset  (reset),
    .stall    (stall),
    .EXsrca    (EXsrca),
	 .EXsrcb    (EXsrcb),
    .EXwritedata    (EXwritedata),
    .EXsignimm    (EXsignimm),
    .EXpcplus4    (EXpcplus4),
    .EXinstr    (EXinstr),
    .EXwritereg    (EXwritereg),
    .EXalucontrol    (EXalucontrol),
    .EXcontrol    (EXcontrol),
    .EXpcrtn    (EXpcrtn));
  //ALU logic
  alu alu(
    .a       (EXsrca),
    .b       (EXsrcb),
    .alucont (EXalucontrol),
    .result  (EXaluout),
    .zero    (EXzero));
//load register between EX stage and MEM stage
//do memory write and read data in MEM stage
  EX #(32) EXtoMA(
    .EXwritedata    (EXwritedata),
    .EXsrca    (EXsrca),
    .EXpcbranch    (EXpcbranch),
    .EXaluout    (EXaluout),
    .EXzero    (EXzero),
    .EXpcsrc    (EXpcsrc),
    .EXpcrtn      (EXpcrtn),
    .EXinstr    (EXinstr),
    .EXpcplus4    (EXpcplus4),
    .EXwritereg    (EXwritereg),
    .EXcontrol    (EXcontrol),
    .clk    (clk),
    .reset  (reset),
    .MAsrca    (MAsrca),
    .MAinstr    (MAinstr),
    .MApcplus4    (MApcplus4),
    .MAwritereg    (MAwritereg),
    .MAcontrol    (MAcontrol),
    .MApcbranch    (MApcbranch),
    .MAaluout    (MAaluout),
    .MAwritedata    (MAwritedata),
    .MAmemwrite (MAmemwrite),
    .MApcrtn    (MApcrtn),
    .MApcsrc    (MApcsrc));
//load register between MA stage and WB stage
//generate some clk to get sync timing for writing
  MA #(32) MAtoWB(
    .MAinstr    (MAinstr),
    .MAaluout    (MAaluout),
    .MApcplus4    (MApcplus4),
    .MAwritereg   (MAwritereg),
    .readdata    (readdata),
    .MAcontrol    (MAcontrol),
    .clk    (clk),
    .reset  (reset),
	 .wtin    (wt),
    .WBinstr    (WBinstr),
    .WBaluout    (WBaluout),
    .WBpcplus4    (WBpcplus4),
    .WBwritereg    (WBwritereg),
    .WBreaddata    (WBreaddata),
    .WBcontrol    (WBcontrol),
	 .wtime    (wtime));
//get final data to write in register and pass signal for write timing 
  mux2_2 #(32) resmux(
    .d0 (WBaluout),
    .d1 (WBreaddata),
	 .sigin (wtime),
    .s  (WBcontrol[1]), //memtoreg
    .y  (reslt),
	 .sigout (_wtime));

  mux2_2 #(32) resjmux(
    .d0 (reslt),
    .d1 (WBpcplus4),
	 .sigin (_wtime),
    .s   (WBcontrol[0]), //jump
    .y   (WBresult),
	 .sigout (wt));

    
endmodule
