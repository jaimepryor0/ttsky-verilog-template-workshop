module vocoder(clk, rst_n, mic, saw_period, out);
// Three-band channel vocoder:
//   mic -> 3 bandpass filters -> rectify -> envelope LPF
//                                            \
//                                             > x  -> sum -> out
//                                            /
//                          sawtooth NCO ----'
// saw_period is the 24-bit NCO phase increment (larger = higher
// pitch, not a literal period). Bandpass filter coefficients are
// placeholders -- replace with values from the Python design.

    input               clk;
    input               rst_n;        // active-low synchronous reset
    input  signed [7:0] mic;
    input        [23:0] saw_period;
    output signed [7:0] out;

    // --- Bandpass filters (TODO: real coefficients from filter design) ----
    wire signed [7:0] bf1_out, bf2_out, bf3_out;

    filter #(.b0(8'sd0), .b1(8'sd0), .b2(8'sd0),
             .a1(8'sd0), .a2(8'sd0))
        bf1 (.clk(clk), .rst_n(rst_n), .in(mic), .out(bf1_out));

    filter #(.b0(8'sd0), .b1(8'sd0), .b2(8'sd0),
             .a1(8'sd0), .a2(8'sd0))
        bf2 (.clk(clk), .rst_n(rst_n), .in(mic), .out(bf2_out));

    filter #(.b0(8'sd0), .b1(8'sd0), .b2(8'sd0),
             .a1(8'sd0), .a2(8'sd0))
        bf3 (.clk(clk), .rst_n(rst_n), .in(mic), .out(bf3_out));

    // --- Rectification (|x|): negate via two's complement if sign bit set
    wire signed [7:0] rect1 = bf1_out[7] ? (~bf1_out + 8'sd1) : bf1_out;
    wire signed [7:0] rect2 = bf2_out[7] ? (~bf2_out + 8'sd1) : bf2_out;
    wire signed [7:0] rect3 = bf3_out[7] ? (~bf3_out + 8'sd1) : bf3_out;

    // --- Envelope: one-pole LPF realised in biquad form ------------------
    // y[n] = a*|x[n]| + (1-a)*y[n-1], with a = 0.125 (Q1.7: 16, 112)
    wire signed [7:0] e1_out, e2_out, e3_out;

    filter #(.b0(8'sd16), .b1(8'sd0), .b2(8'sd0),
             .a1(-8'sd112), .a2(8'sd0))
        env1 (.clk(clk), .rst_n(rst_n), .in(rect1), .out(e1_out));

    filter #(.b0(8'sd16), .b1(8'sd0), .b2(8'sd0),
             .a1(-8'sd112), .a2(8'sd0))
        env2 (.clk(clk), .rst_n(rst_n), .in(rect2), .out(e2_out));

    filter #(.b0(8'sd16), .b1(8'sd0), .b2(8'sd0),
             .a1(-8'sd112), .a2(8'sd0))
        env3 (.clk(clk), .rst_n(rst_n), .in(rect3), .out(e3_out));

    // --- Pitch generator (sawtooth NCO) ----------------------------------
    wire [7:0] pitch_raw;
    pitch pitch_nco (.clk(clk), .rst_n(rst_n),
                     .increment(saw_period), .out(pitch_raw));
    // Reinterpret as bipolar Q1.7 sawtooth (-1.0 .. ~+1.0)
    wire signed [7:0] pitch_sig = $signed(pitch_raw);

    // --- Per-band modulation: pitch * envelope, truncated Q2.14 -> Q1.7 --
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [15:0] m1_full = pitch_sig * e1_out;
    wire signed [15:0] m2_full = pitch_sig * e2_out;
    wire signed [15:0] m3_full = pitch_sig * e3_out;
    /* verilator lint_on UNUSEDSIGNAL */
    wire signed [7:0]  mult1_out = m1_full[14:7];
    wire signed [7:0]  mult2_out = m2_full[14:7];
    wire signed [7:0]  mult3_out = m3_full[14:7];

    // --- Sum bands with 2-bit headroom, then scale back to 8-bit ---------
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [9:0] sum = {{2{mult1_out[7]}}, mult1_out}
                          + {{2{mult2_out[7]}}, mult2_out}
                          + {{2{mult3_out[7]}}, mult3_out};
    /* verilator lint_on UNUSEDSIGNAL */
    assign out = sum[9:2];

endmodule
