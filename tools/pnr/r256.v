(* techmap_celltype = "RAM256X1S" *)
module RAM256X1S_split (O, A, D, WCLK, WE);
  parameter [255:0] INIT = 256'h0;
  parameter [0:0] IS_WCLK_INVERTED = 1'b0;
  output O; input [7:0] A; input D, WCLK, WE;
  wire doA, doB, doC, doD, diA, diB, diC, diD;
  LUT4 #(.INIT(16'hCCCA)) u_diA (.I0(D), .I1(doA), .I2(A[6]), .I3(A[7]), .O(diA));
  LUT4 #(.INIT(16'hCCAC)) u_diB (.I0(D), .I1(doB), .I2(A[6]), .I3(A[7]), .O(diB));
  LUT4 #(.INIT(16'hCACC)) u_diC (.I0(D), .I1(doC), .I2(A[6]), .I3(A[7]), .O(diC));
  LUT4 #(.INIT(16'hACCC)) u_diD (.I0(D), .I1(doD), .I2(A[6]), .I3(A[7]), .O(diD));
  RAM64M #(.INIT_A(INIT[63:0]), .INIT_B(INIT[127:64]), .INIT_C(INIT[191:128]), .INIT_D(INIT[255:192]), .IS_WCLK_INVERTED(IS_WCLK_INVERTED)) m (
    .DOA(doA), .DOB(doB), .DOC(doC), .DOD(doD), .DIA(diA), .DIB(diB), .DIC(diC), .DID(diD),
    .ADDRA(A[5:0]), .ADDRB(A[5:0]), .ADDRC(A[5:0]), .ADDRD(A[5:0]), .WCLK(WCLK), .WE(WE));
  LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_o (.I0(doA), .I1(doB), .I2(doC), .I3(doD), .I4(A[6]), .I5(A[7]), .O(O));
endmodule

(* blackbox *)
module RAM64M (DOA, DOB, DOC, DOD, DIA, DIB, DIC, DID, ADDRA, ADDRB, ADDRC, ADDRD, WCLK, WE);
  parameter [63:0] INIT_A = 64'h0; parameter [63:0] INIT_B = 64'h0; parameter [63:0] INIT_C = 64'h0; parameter [63:0] INIT_D = 64'h0;
  parameter [0:0] IS_WCLK_INVERTED = 1'b0;
  output DOA, DOB, DOC, DOD; input DIA, DIB, DIC, DID; input [5:0] ADDRA, ADDRB, ADDRC, ADDRD; input WCLK, WE;
endmodule
