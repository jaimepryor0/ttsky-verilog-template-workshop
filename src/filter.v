module filter(clk, rst_n, in, out);
// Direct Form II Transposed biquad, normalised (a0 = 1).
// 8-bit signed I/O in Q1.7; coefficients are Q1.7. Each Q2.14
// product is arithmetic-shifted right by FRAC back to Q1.7 to
// match filter_df2_hw in the behavioural model.

    parameter signed [7:0] b0 = 8'sd0;
    parameter signed [7:0] b1 = 8'sd0;
    parameter signed [7:0] b2 = 8'sd0;
    parameter signed [7:0] a1 = 8'sd0;
    parameter signed [7:0] a2 = 8'sd0;
    localparam FRAC = 7;

    input               clk;
    input               rst_n;
    input  signed [7:0] in;
    output signed [7:0] out;

    reg signed [7:0] s1, s2;

    // Full-width signed products (Q2.14). Only bits [14:7] are
    // consumed downstream; synthesis prunes the rest.
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [15:0] in_b0  = in  * b0;
    wire signed [15:0] in_b1  = in  * b1;
    wire signed [15:0] in_b2  = in  * b2;
    wire signed [15:0] out_a1 = out * a1;
    wire signed [15:0] out_a2 = out * a2;
    /* verilator lint_on UNUSEDSIGNAL */

    // Truncate Q2.14 -> Q1.7 by slicing bits [FRAC+7:FRAC]
    // (equivalent to arithmetic-shift-right by FRAC then take low 8 bits)
    wire signed [7:0] xb0 = in_b0 [FRAC+7:FRAC];
    wire signed [7:0] xb1 = in_b1 [FRAC+7:FRAC];
    wire signed [7:0] xb2 = in_b2 [FRAC+7:FRAC];
    wire signed [7:0] ya1 = out_a1[FRAC+7:FRAC];
    wire signed [7:0] ya2 = out_a2[FRAC+7:FRAC];

    assign out = xb0 + s1;

    always @(posedge clk) begin
        if (~rst_n) begin
            s1 <= 0;
            s2 <= 0;
        end else begin
            s1 <= xb1 - ya1 + s2;
            s2 <= xb2 - ya2;
        end
    end
endmodule
