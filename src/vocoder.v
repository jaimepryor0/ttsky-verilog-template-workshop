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
// 8-bit signed throughout: Q1.7 data, Q2.6 coefficients.
//
// Shared serial multiplier: one signed shift-and-add unit handles every
// multiplication in the vocoder (the 5 MACs per biquad x 9 biquads, plus
// the 3 env*saw post-multiplies). Each multiply takes 1 setup cycle + 8
// iteration cycles = 9 chip clocks. Total per audio sample:
//   (6 BF/ENV * 5 phases + 3 SBF * 6 phases) * 9 + 1 = 433 cycles.
//
// Interface:
//   start - 1-cycle pulse to begin processing one sample using the current
//           mic and saw inputs (latched internally on the pulse).
//   done  - 1-cycle pulse on the cycle that `out` first holds the new sample.
//   Further `start` pulses while running are ignored.

    input  wire              clk;
    input  wire              rst_n;
    input  wire              start;
    output reg               done;
    input  wire signed [7:0] mic;
    input  wire signed [7:0] saw;
    output reg  signed [7:0] out;

    // FILTER COEFFICIENTS (Q2.6 signed) //
    parameter signed [7:0] B1_b0 = 8'sd0; // band 1
    parameter signed [7:0] B1_b1 = 8'sd0;
    parameter signed [7:0] B1_b2 = 8'sd0;
    parameter signed [7:0] B1_a1 = 8'sd0;
    parameter signed [7:0] B1_a2 = 8'sd0;

    parameter signed [7:0] B2_b0 = 8'sd0; // band 2
    parameter signed [7:0] B2_b1 = 8'sd0;
    parameter signed [7:0] B2_b2 = 8'sd0;
    parameter signed [7:0] B2_a1 = 8'sd0;
    parameter signed [7:0] B2_a2 = 8'sd0;

    parameter signed [7:0] B3_b0 = 8'sd0; // band 3
    parameter signed [7:0] B3_b1 = 8'sd0;
    parameter signed [7:0] B3_b2 = 8'sd0;
    parameter signed [7:0] B3_a1 = 8'sd0;
    parameter signed [7:0] B3_a2 = 8'sd0;

    // Envelope LPF: one-pole IIR, b = [1-alpha, 0], a = [1, -alpha]
    parameter signed [7:0] ENV_b0 = 8'sd0;
    parameter signed [7:0] ENV_a1 = 8'sd0;

    ///////////////////////////////////////
    // Filter slot indices
    //   0..2 = mic bandpass (B1, B2, B3)
    //   3..5 = envelope LPF (E1, E2, E3) reading rectified bf_y
    //   6..8 = saw bandpass (B1, B2, B3) -- same coeffs as 0..2
    ///////////////////////////////////////

    // --- Latched inputs (captured when start is sampled high) ---
    reg signed [7:0] mic_r, saw_r;

    // --- State RAM: 9 biquads x 2 regs ---
    reg signed [7:0] s1 [0:8];
    reg signed [7:0] s2 [0:8];

    // --- Filter outputs held for downstream consumption ---
    reg signed [7:0] bf_y  [0:2];
    reg signed [7:0] env_y [0:2];
    reg signed [7:0] sbf_y;

    // --- Per-filter MAC intermediates ---
    reg signed [7:0] xb0_r, xb1_r, xb2_r, ya1_r;
    reg signed [7:0] y_r;

    // --- Output accumulator (2 bits of growth for 3-way sum) ---
    reg signed [9:0] acc;

    // --- Control ---
    reg [3:0] filt_idx;   // 0..8
    reg [2:0] mac_phase;  // 0..6 (0..4 = MAC, 5 = post-mul, 6 = output)
    reg [3:0] sub_cnt;    // 0..8: 0 = operand setup, 1..8 = serial-mult iterations
    reg       busy;

    // --- Serial signed multiplier state ---
    // Standard "add and arithmetic-shift-right" with Booth sign-correction on
    // the multiplier's MSB. After 8 iterations the 16-bit product sits in
    // {m_acc[7:0], m_b[7:0]}.
    reg signed [8:0] m_acc;   // 9-bit accumulator (sign extended top of partial product)
    reg        [7:0] m_b;     // multiplier register (B), shifts right each iteration
    reg signed [7:0] m_a;     // multiplicand (A), held throughout

    // Rectify (abs) of mic-BPF outputs -- feeds envelope LPF inputs
    wire signed [7:0] rect0 = bf_y[0][7] ? -bf_y[0] : bf_y[0];
    wire signed [7:0] rect1 = bf_y[1][7] ? -bf_y[1] : bf_y[1];
    wire signed [7:0] rect2 = bf_y[2][7] ? -bf_y[2] : bf_y[2];

    // --- Coefficient ROM (combinational mux on filt_idx) ---
    reg signed [7:0] b0_sel, b1_sel, b2_sel, a1_sel, a2_sel;
    always @* begin
        case (filt_idx)
            4'd0, 4'd6: begin b0_sel=B1_b0; b1_sel=B1_b1; b2_sel=B1_b2; a1_sel=B1_a1; a2_sel=B1_a2; end
            4'd1, 4'd7: begin b0_sel=B2_b0; b1_sel=B2_b1; b2_sel=B2_b2; a1_sel=B2_a1; a2_sel=B2_a2; end
            4'd2, 4'd8: begin b0_sel=B3_b0; b1_sel=B3_b1; b2_sel=B3_b2; a1_sel=B3_a1; a2_sel=B3_a2; end
            4'd3, 4'd4, 4'd5: begin b0_sel=ENV_b0; b1_sel=8'sd0; b2_sel=8'sd0; a1_sel=ENV_a1; a2_sel=8'sd0; end
            default: begin b0_sel=8'sd0; b1_sel=8'sd0; b2_sel=8'sd0; a1_sel=8'sd0; a2_sel=8'sd0; end
        endcase
    end

    // --- Filter input mux ---
    reg signed [7:0] in_sel;
    always @* begin
        case (filt_idx)
            4'd0, 4'd1, 4'd2: in_sel = mic_r;
            4'd3:             in_sel = rect0;
            4'd4:             in_sel = rect1;
            4'd5:             in_sel = rect2;
            4'd6, 4'd7, 4'd8: in_sel = saw_r;
            default:          in_sel = 8'sd0;
        endcase
    end

    // --- Operand selection per phase ---
    //   phase 0..2: in_sel * b0/b1/b2  (Q1.7 * Q2.6)
    //   phase 3..4: y_r    * a1/a2     (Q1.7 * Q2.6)
    //   phase 5  :  env_y[i-6] * sbf_y (Q1.7 * Q1.7)
    reg signed [7:0] op_a, op_b;
    always @* begin
        case (mac_phase)
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
                    default: begin op_a = 8'sd0;    op_b = 8'sd0; end
                endcase
            end
            default: begin op_a = 8'sd0; op_b = 8'sd0; end
        endcase
    end

    // --- Serial multiplier iteration (combinational) ---
    // is_last_iter: this cycle holds the multiplier's MSB (sign bit) in m_b[0].
    // On that cycle we SUBTRACT instead of ADD -- this turns the unsigned
    // sequential algorithm into a signed one (Booth's MSB correction).
    wire is_last_iter = (sub_cnt == 4'd8);
    wire signed [8:0] m_a_sext = {m_a[7], m_a};
    reg  signed [8:0] m_new_top;
    always @* begin
        if (m_b[0]) begin
            if (is_last_iter) m_new_top = m_acc - m_a_sext;
            else              m_new_top = m_acc + m_a_sext;
        end else
            m_new_top = m_acc;
    end

    // Post-shift values of acc and b (combinational; what gets latched on
    // an iteration cycle).
    wire signed [8:0] m_acc_after = {m_new_top[8], m_new_top[8:1]};
    wire        [7:0] m_b_after   = {m_new_top[0], m_b[7:1]};

    // Full 16-bit product, valid at the end of sub_cnt=8. Slice differently
    // for the filter MAC (Q3.13 -> Q1.7) vs the env*saw post-mul (Q2.14 -> Q1.7).
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [15:0] mul_prod = {m_acc_after[7:0], m_b_after[7:0]};
    /* verilator lint_on UNUSEDSIGNAL */
    wire signed [7:0] prod_trunc_filt   = mul_prod[13:6];
    wire signed [7:0] prod_trunc_envsaw = mul_prod[14:7];

    // Sign-extend the post-multiply product to the 10-bit accumulator width
    wire signed [9:0] sxprod = {{2{prod_trunc_envsaw[7]}}, prod_trunc_envsaw};

    // --- FSM ---
    integer i;
    always @(posedge clk) begin
        if (~rst_n) begin
            for (i = 0; i < 9; i = i + 1) begin
                s1[i] <= 8'sd0;
                s2[i] <= 8'sd0;
            end
            for (i = 0; i < 3; i = i + 1) begin
                bf_y[i]  <= 8'sd0;
                env_y[i] <= 8'sd0;
            end
            sbf_y     <= 8'sd0;
            xb0_r     <= 8'sd0;
            xb1_r     <= 8'sd0;
            xb2_r     <= 8'sd0;
            ya1_r     <= 8'sd0;
            y_r       <= 8'sd0;
            acc       <= 10'sd0;
            mic_r     <= 8'sd0;
            saw_r     <= 8'sd0;
            filt_idx  <= 4'd0;
            mac_phase <= 3'd0;
            sub_cnt   <= 4'd0;
            busy      <= 1'b0;
            done      <= 1'b0;
            out       <= 8'sd0;
            m_acc     <= 9'sd0;
            m_b       <= 8'd0;
            m_a       <= 8'sd0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    busy      <= 1'b1;
                    filt_idx  <= 4'd0;
                    mac_phase <= 3'd0;
                    sub_cnt   <= 4'd0;
                    mic_r     <= mic;
                    saw_r     <= saw;
                    acc       <= 10'sd0;
                end
            end else begin
                if (mac_phase == 3'd6) begin
                    // Output phase: register out, pulse done, return to idle.
                    out       <= acc[7:0];
                    done      <= 1'b1;
                    busy      <= 1'b0;
                    filt_idx  <= 4'd0;
                    mac_phase <= 3'd0;
                    sub_cnt   <= 4'd0;
                end else begin
                    // MAC phase (0..4) or post-mul phase (5). Each spans
                    // sub_cnt = 0 (setup) followed by sub_cnt = 1..8
                    // (8 multiplier iterations).
                    if (sub_cnt == 4'd0) begin
                        // Setup: latch operands, clear accumulator, begin iterations.
                        m_acc   <= 9'sd0;
                        m_b     <= op_b;
                        m_a     <= op_a;
                        sub_cnt <= 4'd1;
                    end else if (sub_cnt != 4'd8) begin
                        // Iteration (not last): update {acc, b} = arith-shift-right
                        m_acc   <= m_acc_after;
                        m_b     <= m_b_after;
                        sub_cnt <= sub_cnt + 4'd1;
                    end else begin
                        // sub_cnt == 8: final iteration. Latch final shift and
                        // capture the resulting 16-bit product.
                        m_acc   <= m_acc_after;
                        m_b     <= m_b_after;
                        sub_cnt <= 4'd0;

                        // Phase-specific bookkeeping and transition.
                        if (mac_phase == 3'd5) begin
                            // Post-mul: accumulate env*sbf into output sum.
                            acc <= acc + sxprod;
                            if (filt_idx == 4'd8) begin
                                mac_phase <= 3'd6;
                            end else begin
                                filt_idx  <= filt_idx + 4'd1;
                                mac_phase <= 3'd0;
                            end
                        end else begin
                            // MAC phase 0..4 -- capture product and update state.
                            case (mac_phase)
                                3'd0: xb0_r <= prod_trunc_filt;
                                3'd1: begin
                                    xb1_r <= prod_trunc_filt;
                                    y_r   <= xb0_r + s1[filt_idx];  // y[n] = b0*x + s1
                                end
                                3'd2: xb2_r <= prod_trunc_filt;
                                3'd3: ya1_r <= prod_trunc_filt;
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
                                end
                                default: ;
                            endcase

                            if (mac_phase < 3'd4) begin
                                mac_phase <= mac_phase + 3'd1;
                            end else begin
                                // mac_phase == 4: end of biquad
                                if (filt_idx >= 4'd6) begin
                                    mac_phase <= 3'd5;  // SBF needs post-mul
                                end else begin
                                    filt_idx  <= filt_idx + 4'd1;
                                    mac_phase <= 3'd0;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
endmodule
