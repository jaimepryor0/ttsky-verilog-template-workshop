`default_nettype none

module vocoder(clk, rst_n, start, done, mic, saw, out);
// Three-band channel vocoder, matching vocoder_fixed_point.py exactly.
//
//   mic -> 3 bandpass filters -> |.| -> envelope LPF -.
//                                                      \
//                                                       > x -> sum -> out
//                                                      /
//   saw -> the same 3 bandpass filters ----------------'
//
// 16-bit signed throughout: Q1.15 data, Q2.14 coefficients.
//
// Shared-MAC implementation: a single 16x16 signed multiplier is sequenced
// over the 9 internal biquads (3 mic BPF + 3 envelope LPF + 3 saw BPF) plus
// the 3 envelope*saw post-multiplies. 49 chip clocks per audio sample.
//
// Interface:
//   start - 1-cycle pulse to begin processing one sample using the current
//           mic and saw inputs (latched internally on the pulse).
//   done  - 1-cycle pulse on the cycle that `out` first holds the new sample.
//   busy is implicit; further `start` pulses while running are ignored.

    input  wire               clk;
    input  wire               rst_n;
    input  wire               start;
    output reg                done;
    input  wire signed [15:0] mic;
    input  wire signed [15:0] saw;
    output reg  signed [15:0] out;

    // FILTER COEFFICIENTS (Q2.14 signed) //
    parameter signed [15:0] B1_b0 = 16'sd0; // band 1
    parameter signed [15:0] B1_b1 = 16'sd0;
    parameter signed [15:0] B1_b2 = 16'sd0;
    parameter signed [15:0] B1_a1 = 16'sd0;
    parameter signed [15:0] B1_a2 = 16'sd0;

    parameter signed [15:0] B2_b0 = 16'sd0; // band 2
    parameter signed [15:0] B2_b1 = 16'sd0;
    parameter signed [15:0] B2_b2 = 16'sd0;
    parameter signed [15:0] B2_a1 = 16'sd0;
    parameter signed [15:0] B2_a2 = 16'sd0;

    parameter signed [15:0] B3_b0 = 16'sd0; // band 3
    parameter signed [15:0] B3_b1 = 16'sd0;
    parameter signed [15:0] B3_b2 = 16'sd0;
    parameter signed [15:0] B3_a1 = 16'sd0;
    parameter signed [15:0] B3_a2 = 16'sd0;

    // Envelope LPF: one-pole IIR, b = [1-alpha, 0], a = [1, -alpha]
    parameter signed [15:0] ENV_b0 = 16'sd0;
    parameter signed [15:0] ENV_a1 = 16'sd0;

    ///////////////////////////////////////
    // Filter slot indices
    //   0..2 = mic bandpass (B1, B2, B3)
    //   3..5 = envelope LPF (E1, E2, E3) reading rectified bf_y
    //   6..8 = saw bandpass (B1, B2, B3) -- same coeffs as 0..2
    ///////////////////////////////////////

    // --- Latched inputs (captured when start is sampled high) ---
    reg signed [15:0] mic_r, saw_r;

    // --- State RAM: 9 biquads x 2 regs = 36 bytes ---
    reg signed [15:0] s1 [0:8];
    reg signed [15:0] s2 [0:8];

    // --- Filter outputs held for downstream consumption ---
    reg signed [15:0] bf_y  [0:2];   // mic BPF outputs (feed envelope LPFs)
    reg signed [15:0] env_y [0:2];   // envelope outputs (feed final multiplies)
    reg signed [15:0] sbf_y;         // current saw BPF output (consumed immediately)

    // --- Per-filter MAC intermediates ---
    reg signed [15:0] xb0_r, xb1_r, xb2_r, ya1_r;
    reg signed [15:0] y_r;

    // --- Output accumulator ---
    reg signed [17:0] acc;

    // --- Control ---
    reg [3:0] filt_idx;   // 0..8
    reg [2:0] phase;      // 0..6
    reg       busy;

    // Rectify (abs) of mic-BPF outputs -- feeds envelope LPF inputs
    wire signed [15:0] rect0 = bf_y[0][15] ? -bf_y[0] : bf_y[0];
    wire signed [15:0] rect1 = bf_y[1][15] ? -bf_y[1] : bf_y[1];
    wire signed [15:0] rect2 = bf_y[2][15] ? -bf_y[2] : bf_y[2];

    // --- Coefficient ROM (combinational mux on filt_idx) ---
    reg signed [15:0] b0_sel, b1_sel, b2_sel, a1_sel, a2_sel;
    always @* begin
        case (filt_idx)
            4'd0, 4'd6: begin b0_sel=B1_b0; b1_sel=B1_b1; b2_sel=B1_b2; a1_sel=B1_a1; a2_sel=B1_a2; end
            4'd1, 4'd7: begin b0_sel=B2_b0; b1_sel=B2_b1; b2_sel=B2_b2; a1_sel=B2_a1; a2_sel=B2_a2; end
            4'd2, 4'd8: begin b0_sel=B3_b0; b1_sel=B3_b1; b2_sel=B3_b2; a1_sel=B3_a1; a2_sel=B3_a2; end
            4'd3, 4'd4, 4'd5: begin b0_sel=ENV_b0; b1_sel=16'sd0; b2_sel=16'sd0; a1_sel=ENV_a1; a2_sel=16'sd0; end
            default: begin b0_sel=16'sd0; b1_sel=16'sd0; b2_sel=16'sd0; a1_sel=16'sd0; a2_sel=16'sd0; end
        endcase
    end

    // --- Filter input mux ---
    reg signed [15:0] in_sel;
    always @* begin
        case (filt_idx)
            4'd0, 4'd1, 4'd2: in_sel = mic_r;
            4'd3:             in_sel = rect0;
            4'd4:             in_sel = rect1;
            4'd5:             in_sel = rect2;
            4'd6, 4'd7, 4'd8: in_sel = saw_r;
            default:          in_sel = 16'sd0;
        endcase
    end

    // --- Shared multiplier ---
    // op_a/op_b vary by phase:
    //   phase 0..2: in_sel * b0/b1/b2  (Q1.15 * Q2.14)
    //   phase 3..4: y_r    * a1/a2     (Q1.15 * Q2.14)
    //   phase 5  :  env_y[i-6] * sbf_y (Q1.15 * Q1.15)
    reg signed [15:0] op_a, op_b;
    always @* begin
        case (phase)
            3'd0: begin op_a = in_sel; op_b = b0_sel; end
            3'd1: begin op_a = in_sel; op_b = b1_sel; end
            3'd2: begin op_a = in_sel; op_b = b2_sel; end
            3'd3: begin op_a = y_r;    op_b = a1_sel; end
            3'd4: begin op_a = y_r;    op_b = a2_sel; end
            3'd5: begin
                case (filt_idx)
                    4'd6:    begin op_a = env_y[0]; op_b = sbf_y; end
                    4'd7:    begin op_a = env_y[1]; op_b = sbf_y; end
                    4'd8:    begin op_a = env_y[2]; op_b = sbf_y; end
                    default: begin op_a = 16'sd0;   op_b = 16'sd0; end
                endcase
            end
            default: begin op_a = 16'sd0; op_b = 16'sd0; end
        endcase
    end

    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [31:0] prod = op_a * op_b;
    /* verilator lint_on UNUSEDSIGNAL */
    // Filter MAC: Q1.15 * Q2.14 = Q3.29 in 32 bits; take [29:14] for Q1.15
    wire signed [15:0] prod_trunc_filt   = prod[29:14];
    // Env*saw mult: Q1.15 * Q1.15 = Q2.30 in 32 bits; take [30:15] for Q1.15
    wire signed [15:0] prod_trunc_envsaw = prod[30:15];

    // Sign-extend the post-multiply product to the 18-bit accumulator width
    wire signed [17:0] sxprod = {{2{prod_trunc_envsaw[15]}}, prod_trunc_envsaw};

    // --- FSM ---
    integer i;
    always @(posedge clk) begin
        if (~rst_n) begin
            for (i = 0; i < 9; i = i + 1) begin
                s1[i] <= 16'sd0;
                s2[i] <= 16'sd0;
            end
            for (i = 0; i < 3; i = i + 1) begin
                bf_y[i]  <= 16'sd0;
                env_y[i] <= 16'sd0;
            end
            sbf_y    <= 16'sd0;
            xb0_r    <= 16'sd0;
            xb1_r    <= 16'sd0;
            xb2_r    <= 16'sd0;
            ya1_r    <= 16'sd0;
            y_r      <= 16'sd0;
            acc      <= 18'sd0;
            mic_r    <= 16'sd0;
            saw_r    <= 16'sd0;
            filt_idx <= 4'd0;
            phase    <= 3'd0;
            busy     <= 1'b0;
            done     <= 1'b0;
            out      <= 16'sd0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    busy     <= 1'b1;
                    filt_idx <= 4'd0;
                    phase    <= 3'd0;
                    mic_r    <= mic;
                    saw_r    <= saw;
                    acc      <= 18'sd0;
                end
            end else begin
                case (phase)
                    3'd0: begin
                        xb0_r <= prod_trunc_filt;
                        phase <= 3'd1;
                    end
                    3'd1: begin
                        xb1_r <= prod_trunc_filt;
                        // y_r = b0*x + s1 = y[n]   (matches filter_df2_hw)
                        y_r   <= xb0_r + s1[filt_idx];
                        phase <= 3'd2;
                    end
                    3'd2: begin
                        xb2_r <= prod_trunc_filt;
                        phase <= 3'd3;
                    end
                    3'd3: begin
                        ya1_r <= prod_trunc_filt;
                        phase <= 3'd4;
                    end
                    3'd4: begin
                        // ya2 == prod_trunc_filt this cycle
                        s1[filt_idx] <= xb1_r - ya1_r + s2[filt_idx];
                        s2[filt_idx] <= xb2_r - prod_trunc_filt;
                        case (filt_idx)
                            4'd0:             bf_y[0]  <= y_r;
                            4'd1:             bf_y[1]  <= y_r;
                            4'd2:             bf_y[2]  <= y_r;
                            4'd3:             env_y[0] <= y_r;
                            4'd4:             env_y[1] <= y_r;
                            4'd5:             env_y[2] <= y_r;
                            4'd6, 4'd7, 4'd8: sbf_y    <= y_r;
                            default: ;
                        endcase
                        if (filt_idx >= 4'd6) begin
                            phase <= 3'd5;   // saw BPF: need post-mul phase
                        end else begin
                            filt_idx <= filt_idx + 4'd1;
                            phase    <= 3'd0;
                        end
                    end
                    3'd5: begin
                        // Accumulate env*sbf for this band
                        acc <= acc + sxprod;
                        if (filt_idx == 4'd8) begin
                            phase <= 3'd6;
                        end else begin
                            filt_idx <= filt_idx + 4'd1;
                            phase    <= 3'd0;
                        end
                    end
                    3'd6: begin
                        // Final cycle: register output, pulse done
                        out      <= acc[15:0];
                        done     <= 1'b1;
                        busy     <= 1'b0;
                        filt_idx <= 4'd0;
                        phase    <= 3'd0;
                    end
                    default: phase <= 3'd0;
                endcase
            end
        end
    end
endmodule
