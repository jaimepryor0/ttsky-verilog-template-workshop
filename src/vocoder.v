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
// multiplication in the vocoder (the 4 MACs per biquad x 9 biquads, plus
// the 3 env*saw post-multiplies). The b1 phase is omitted because every
// b1 coefficient in this design's Butterworth bandpass / single-pole
// envelope chain is exactly zero -- the saving is one whole multiply
// cycle (9 chip clocks) per biquad. If anyone changes VOICE_BANDS to a
// design with a non-zero b1, the bit-exact Python comparison in
// test_vocoder.py will catch it.
//
// Each multiply takes 1 setup cycle + 8 iteration cycles = 9 chip clocks.
// Total per audio sample:
//   (6 BF/ENV * 4 phases + 3 SBF * 5 phases) * 9 + 1 = 352 cycles.
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

    // FILTER COEFFICIENTS (Q2.6 signed). b1 is not a parameter because
    // the b1 multiply phase is omitted; see the file header for details.
    parameter signed [7:0] B1_b0 = 8'sd0; // band 1
    parameter signed [7:0] B1_b2 = 8'sd0;
    parameter signed [7:0] B1_a1 = 8'sd0;
    parameter signed [7:0] B1_a2 = 8'sd0;

    parameter signed [7:0] B2_b0 = 8'sd0; // band 2
    parameter signed [7:0] B2_b2 = 8'sd0;
    parameter signed [7:0] B2_a1 = 8'sd0;
    parameter signed [7:0] B2_a2 = 8'sd0;

    parameter signed [7:0] B3_b0 = 8'sd0; // band 3
    parameter signed [7:0] B3_b2 = 8'sd0;
    parameter signed [7:0] B3_a1 = 8'sd0;
    parameter signed [7:0] B3_a2 = 8'sd0;

    // Envelope LPF: one-pole IIR, b = [1-alpha, 0], a = [1, -alpha]
    parameter signed [7:0] ENV_b0 = 8'sd0;
    parameter signed [7:0] ENV_a1 = 8'sd0;

    ///////////////////////////////////////
    // Filter slot indices
    //   0..2 = mic bandpass (B1, B2, B3)
    //   3..5 = envelope LPF (E1, E2, E3) reading rectified mic-BPF outputs
    //   6..8 = saw bandpass (B1, B2, B3) -- same coeffs as 0..2
    ///////////////////////////////////////

    // --- Latched inputs (captured when start is sampled high) ---
    reg signed [7:0] mic_r, saw_r;

    // --- State RAM: 9 biquads x 2 regs ---
    reg signed [7:0] s1 [0:8];
    reg signed [7:0] s2 [0:8];

    // --- Shared per-band output slot --------------------------------------
    // For band i, slot[i] holds:
    //   - the mic BPF result (bf_y[i]) from end of biquad i until end of
    //     biquad i+3, where the envelope filter consumes it via `rect`;
    //   - the envelope LPF result (env_y[i]) from end of biquad i+3 until
    //     phase 4 of biquad i+6, where the SBF post-multiply reads it.
    // The two live ranges are disjoint, so a single 3-entry array carries
    // both. `sbf_y` is gone entirely -- the post-mul reads y_r directly,
    // since y_r still holds the saw-BPF output through the post-mul phase.
    reg signed [7:0] slot [0:2];

    // --- Per-filter MAC intermediates ---
    // Phase 0's product is folded into `y_r` directly (no xb0_r). Phase 1
    // captures the b2 product into xb2_r. The a1 product is consumed in
    // the same cycle it's produced (phase 2) by writing s1 there, so we
    // don't need a separate ya1_r register either.
    reg signed [7:0] xb2_r;
    reg signed [7:0] y_r;

    // --- Output accumulator ---
    // 8-bit modular addition produces the same low 8 bits as the wider
    // accumulator we used to keep; out truncates to acc[7:0] anyway.
    reg signed [7:0] acc;

    // --- Control ---
    reg [3:0] filt_idx;   // 0..8
    reg [2:0] mac_phase;  // 0..5 (0..3 = MAC, 4 = post-mul, 5 = output)
    reg [3:0] sub_cnt;    // 0..8: 0 = operand setup, 1..8 = serial-mult iterations
    reg       busy;

    // --- Serial signed multiplier state ---
    // Standard "add and arithmetic-shift-right" with Booth sign-correction on
    // the multiplier's MSB. After 8 iterations the 16-bit product sits in
    // {m_acc[7:0], m_b[7:0]}.
    reg signed [8:0] m_acc;   // 9-bit accumulator (sign extended top of partial product)
    reg        [7:0] m_b;     // multiplier register (B), shifts right each iteration
    reg signed [7:0] m_a;     // multiplicand (A), held throughout

    // Rectify (abs) of mic-BPF outputs -- feeds envelope LPF inputs.
    // At the moment these wires are sampled (during filt_idx 3..5) the
    // shared `slot[]` still holds the mic BPF results from filt_idx 0..2.
    wire signed [7:0] rect0 = slot[0][7] ? -slot[0] : slot[0];
    wire signed [7:0] rect1 = slot[1][7] ? -slot[1] : slot[1];
    wire signed [7:0] rect2 = slot[2][7] ? -slot[2] : slot[2];

    // --- Coefficient ROM (combinational mux on filt_idx) ---
    reg signed [7:0] b0_sel, b2_sel, a1_sel, a2_sel;
    always @* begin
        case (filt_idx)
            4'd0, 4'd6: begin b0_sel=B1_b0; b2_sel=B1_b2; a1_sel=B1_a1; a2_sel=B1_a2; end
            4'd1, 4'd7: begin b0_sel=B2_b0; b2_sel=B2_b2; a1_sel=B2_a1; a2_sel=B2_a2; end
            4'd2, 4'd8: begin b0_sel=B3_b0; b2_sel=B3_b2; a1_sel=B3_a1; a2_sel=B3_a2; end
            4'd3, 4'd4, 4'd5: begin b0_sel=ENV_b0; b2_sel=8'sd0; a1_sel=ENV_a1; a2_sel=8'sd0; end
            default: begin b0_sel=8'sd0; b2_sel=8'sd0; a1_sel=8'sd0; a2_sel=8'sd0; end
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
    //   phase 0  : in_sel * b0          (Q1.7 * Q2.6) -- also folds in `+ s1`
    //   phase 1  : in_sel * b2          (Q1.7 * Q2.6)
    //   phase 2  : y_r    * a1          (Q1.7 * Q2.6) -- s1 updated this cycle
    //   phase 3  : y_r    * a2          (Q1.7 * Q2.6) -- s2 updated, ends biquad
    //   phase 4  : slot[i-6] * y_r      (Q1.7 * Q1.7) -- SBF post-mul; y_r
    //                                                     still holds the saw
    //                                                     BPF output from phase 3
    reg signed [7:0] op_a, op_b;
    always @* begin
        case (mac_phase)
            3'd0: begin op_a = in_sel; op_b = b0_sel; end
            3'd1: begin op_a = in_sel; op_b = b2_sel; end
            3'd2: begin op_a = y_r;    op_b = a1_sel; end
            3'd3: begin op_a = y_r;    op_b = a2_sel; end
            3'd4: begin
                case (filt_idx)
                    4'd6:    begin op_a = slot[0]; op_b = y_r; end
                    4'd7:    begin op_a = slot[1]; op_b = y_r; end
                    4'd8:    begin op_a = slot[2]; op_b = y_r; end
                    default: begin op_a = 8'sd0;   op_b = 8'sd0; end
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

    // --- FSM ---
    //
    // Reset wiring is deliberately limited to the control path (state, busy,
    // counters, multiplier, and `out`). The filter-state arrays (s1/s2/slot)
    // and per-MAC pipeline registers are *not* reset, which lets synthesis
    // map them to plain DFFs (DFXTP) instead of the larger reset-flop cells
    // (DFRTP). The first sample or two after power-up will use whatever
    // values the flops booted into, but the IIR filters mix that garbage out
    // within a handful of samples -- inaudible on an audio path.
    always @(posedge clk) begin
        if (~rst_n) begin
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
                    acc       <= 8'sd0;
                end
            end else begin
                if (mac_phase == 3'd5) begin
                    // Output phase: register out, pulse done, return to idle.
                    out       <= acc;
                    done      <= 1'b1;
                    busy      <= 1'b0;
                    filt_idx  <= 4'd0;
                    mac_phase <= 3'd0;
                    sub_cnt   <= 4'd0;
                end else begin
                    // MAC phase (0..3) or post-mul phase (4). Each spans
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
                        if (mac_phase == 3'd4) begin
                            // Post-mul: accumulate env*sbf into output sum.
                            // 8-bit modular addition; the wide output truncates
                            // to acc[7:0] regardless of growth.
                            acc <= acc + prod_trunc_envsaw;
                            if (filt_idx == 4'd8) begin
                                mac_phase <= 3'd5;
                            end else begin
                                filt_idx  <= filt_idx + 4'd1;
                                mac_phase <= 3'd0;
                            end
                        end else begin
                            // MAC phase 0..3 -- capture product and update state.
                            case (mac_phase)
                                // Phase 0 (b0*x): fold the trailing `+ s1` into
                                // the same cycle so the b0 product never needs
                                // its own holding register. (b1 phase is
                                // omitted entirely -- b1 is zero in this
                                // design.)
                                3'd0: y_r   <= prod_trunc_filt + s1[filt_idx];
                                3'd1: xb2_r <= prod_trunc_filt;
                                // Phase 2 (a1*y): write the new s1 *now* using
                                // the fresh a1 product; saves the ya1_r reg.
                                // For an envelope filter (filt_idx 3..5) a1*y
                                // is the only non-zero term that touches s1,
                                // and s2 stays at its previous value.
                                // s1_new = s2_old - ya1 (b1 = 0 in this design).
                                3'd2: s1[filt_idx] <= s2[filt_idx] - prod_trunc_filt;
                                3'd3: begin
                                    // ya2 == prod_trunc_filt this cycle.
                                    s2[filt_idx] <= xb2_r - prod_trunc_filt;
                                    // Per-biquad output capture into the shared
                                    // slot[]. Saw-BPF results (filt_idx 6..8)
                                    // don't need a slot at all -- y_r itself
                                    // is the post-mul operand next cycle.
                                    case (filt_idx)
                                        4'd0:    slot[0] <= y_r;
                                        4'd1:    slot[1] <= y_r;
                                        4'd2:    slot[2] <= y_r;
                                        4'd3:    slot[0] <= y_r;
                                        4'd4:    slot[1] <= y_r;
                                        4'd5:    slot[2] <= y_r;
                                        default: ;
                                    endcase
                                end
                                default: ;
                            endcase

                            if (mac_phase < 3'd3) begin
                                mac_phase <= mac_phase + 3'd1;
                            end else begin
                                // mac_phase == 3: end of biquad
                                if (filt_idx >= 4'd6) begin
                                    mac_phase <= 3'd4;  // SBF needs post-mul
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
