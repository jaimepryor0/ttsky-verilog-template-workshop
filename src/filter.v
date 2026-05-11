`default_nettype none

module filter(clk, rst_n, en, in, out);
// Direct Form II Transposed biquad, normalised (a0 = 1).
//
// Matches filter_df2_hw in src/filter_df2.py:
//   yn      = trunc(in*b0)
//   out     = trunc(yn + s1)
//   s1_next = trunc( trunc(in*b1) - trunc(out*a1) + s2 )
//   s2_next = trunc( trunc(in*b2) - trunc(out*a2) )
//
// Data is Q1.15 (16-bit signed). Coefficients are Q2.14 (16-bit
// signed). Each Q3.29 product is sliced [29:14] to bring it back
// to Q1.15 -- matches hw_int.truncate(16, 15).

    parameter DATA_W    = 16;
    parameter COEF_W    = 16;
    parameter COEF_FRAC = 14;
    localparam PROD_W   = DATA_W + COEF_W;

    parameter signed [COEF_W-1:0] b0 = 0;
    parameter signed [COEF_W-1:0] b1 = 0;
    parameter signed [COEF_W-1:0] b2 = 0;
    parameter signed [COEF_W-1:0] a1 = 0;
    parameter signed [COEF_W-1:0] a2 = 0;

    input  wire                     clk;
    input  wire                     rst_n;
    input  wire                     en;      // 1-cycle pulse to advance state
    input  wire signed [DATA_W-1:0] in;
    output wire signed [DATA_W-1:0] out;

    reg signed [DATA_W-1:0] s1; // state register 1
    reg signed [DATA_W-1:0] s2; // state register 2

    // Full-width signed products. Only bits [COEF_FRAC + DATA_W - 1 : COEF_FRAC]
    // are consumed downstream; synthesis prunes the rest.
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [PROD_W-1:0] in_b0  = in  * b0;
    wire signed [PROD_W-1:0] in_b1  = in  * b1;
    wire signed [PROD_W-1:0] in_b2  = in  * b2;
    wire signed [PROD_W-1:0] out_a1 = out * a1;
    wire signed [PROD_W-1:0] out_a2 = out * a2;
    /* verilator lint_on UNUSEDSIGNAL */

    // Truncate each Q3.29 product back to Q1.15
    wire signed [DATA_W-1:0] xb0 = in_b0 [COEF_FRAC+DATA_W-1:COEF_FRAC];
    wire signed [DATA_W-1:0] xb1 = in_b1 [COEF_FRAC+DATA_W-1:COEF_FRAC];
    wire signed [DATA_W-1:0] xb2 = in_b2 [COEF_FRAC+DATA_W-1:COEF_FRAC];
    wire signed [DATA_W-1:0] ya1 = out_a1[COEF_FRAC+DATA_W-1:COEF_FRAC];
    wire signed [DATA_W-1:0] ya2 = out_a2[COEF_FRAC+DATA_W-1:COEF_FRAC];

    // Assign output
    assign out = xb0 + s1;

    always @(posedge clk) begin // synchronous reset, clock-enabled
        if (~rst_n) begin
            s1 <= 0;
            s2 <= 0;
        end else if (en) begin
            // Update state registers
            s1 <= xb1 - ya1 + s2;
            s2 <= xb2 - ya2;
        end
    end
endmodule
