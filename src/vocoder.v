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
// multiplication in the vocoder. The phase layout exploits two structural
// zeros in the coefficient set:
//
//   * b1 is zero for every biquad -- holds because the Butterworth
//     bandpass at FS=48k quantises b1 to 0, and the envelope LPF is
//     single-pole. The b1 multiply phase is therefore omitted across the
//     whole pipeline. test_vocoder._design_parameters() refuses to build
//     if a future band design produces a non-zero b1.
//
//   * b2 and a2 are zero for the envelope LPFs (filt_idx 3..5) by
//     topology -- a one-pole IIR has no second-order taps. The b2*x and
//     a2*y phases are skipped for those three slots, so each envelope
//     biquad runs only 2 multiplies (b0*x, a1*y) instead of 4. s2[3..5]
//     is never written and gets pruned by synthesis.
//
// Each multiply takes 1 setup cycle + 8 iteration cycles = 9 chip clocks.
// Total per audio sample:
//   (3 BPF * 4 phases + 3 ENV * 2 phases + 3 SBF * 5 phases) * 9 + 1 = 298 cycles.
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

    // --- State RAM ---
    // s1 has one entry per biquad slot, including the envelope filters
    // (which use s1 normally). s2 is only used by the second-order biquads
    // (3 BPFs + 3 SBFs = 6 slots) -- the envelope filters are one-pole so
    // their s2 would be permanently zero. We index s2 with a compressed
    // address (`s2_addr`, below) instead of the full 9-entry filt_idx.
    reg signed [7:0] s1 [0:8];
    reg signed [7:0] s2 [0:5];

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

    // --- Control ---
    reg [3:0] filt_idx;   // 0..8
    reg [2:0] mac_phase;  // 0..4 (0..3 = MAC, 4 = post-mul / final accumulate)
    reg [3:0] sub_cnt;    // 0..8: 0 = operand setup, 1..8 = serial-mult iterations
    reg       busy;

    // The output register doubles as the per-sample post-mul accumulator: it
    // gets cleared to 0 on `start`, accumulates env*sbf products at phase 4,
    // and is left holding the new sample the moment `done` pulses.

    // --- Serial signed multiplier state ---
    // Standard "add and arithmetic-shift-right" with Booth sign-correction on
    // the multiplier's MSB. After 8 iterations the 16-bit product sits in
    // {m_acc, m_b}. m_acc is stored at 8 bits and sign-extended to 9 bits
    // on the way into the add; the high bit of the 9-bit partial sum is
    // always the same as bit 7 after each arith-shift-right, so storing the
    // extra bit would be redundant. m_a is not registered -- op_a is stable
    // across the multiplier's 9 cycles (mac_phase / filt_idx only change at
    // sub_cnt==8), so it's fed combinationally.
    reg signed [7:0] m_acc;   // upper half of the partial product
    reg        [7:0] m_b;     // multiplier register (B), shifts right each iteration

    // Rectify (abs) of mic-BPF outputs -- feeds envelope LPF inputs.
    // At the moment these wires are sampled (during filt_idx 3..5) the
    // shared `slot[]` still holds the mic BPF results from filt_idx 0..2.
    wire signed [7:0] rect0 = slot[0][7] ? -slot[0] : slot[0];
    wire signed [7:0] rect1 = slot[1][7] ? -slot[1] : slot[1];
    wire signed [7:0] rect2 = slot[2][7] ? -slot[2] : slot[2];

    // True while we're processing one of the envelope LPF slots.
    wire is_env = (filt_idx >= 4'd3) && (filt_idx <= 4'd5);

    // Compress filt_idx {0,1,2,6,7,8} -> s2 address {0,1,2,3,4,5}. The
    // envelope filters (filt_idx 3..5) never touch s2, so we don't care
    // what this evaluates to for them. filt_idx=8 wraps in the 3-bit
    // subtract: 000 - 011 = 101 = 5, exactly what we want.
    wire [2:0] s2_addr = (filt_idx >= 4'd6) ? (filt_idx[2:0] - 3'd3)
                                            :  filt_idx[2:0];

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
    //   phase 1  : in_sel * b2          (Q1.7 * Q2.6) -- skipped for env
    //   phase 2  : y_r    * a1          (Q1.7 * Q2.6) -- s1 updated this cycle
    //   phase 3  : y_r    * a2          (Q1.7 * Q2.6) -- s2 updated; skipped for env
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
    wire signed [8:0] m_a_sext   = {op_a[7], op_a};
    wire signed [8:0] m_acc_sext = {m_acc[7], m_acc};
    reg  signed [8:0] m_new_top;
    always @* begin
        if (m_b[0]) begin
            if (is_last_iter) m_new_top = m_acc_sext - m_a_sext;
            else              m_new_top = m_acc_sext + m_a_sext;
        end else
            m_new_top = m_acc_sext;
    end

    // Post-shift values of acc and b (combinational; what gets latched on
    // an iteration cycle). m_new_top[8:1] is the arith-shifted result --
    // its MSB is the sign of the partial sum, which we re-extend on the
    // next iteration via m_acc_sext above.
    wire signed [7:0] m_acc_after = m_new_top[8:1];
    wire        [7:0] m_b_after   = {m_new_top[0], m_b[7:1]};

    // Full 16-bit product, valid at the end of sub_cnt=8. Slice differently
    // for the filter MAC (Q3.13 -> Q1.7) vs the env*saw post-mul (Q2.14 -> Q1.7).
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [15:0] mul_prod = {m_acc_after, m_b_after};
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
            m_acc     <= 8'sd0;
            m_b       <= 8'd0;
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
                    out       <= 8'sd0;     // also the post-mul accumulator
                end
            end else begin
                // MAC phase (0..3) or post-mul phase (4). Each spans
                // sub_cnt = 0 (setup) followed by sub_cnt = 1..8 (8 iter).
                if (sub_cnt == 4'd0) begin
                    // Setup: latch the multiplier register, clear accumulator,
                    // begin iterations. (m_a is no longer registered -- op_a
                    // is read combinationally throughout the multiply.)
                    m_acc   <= 8'sd0;
                    m_b     <= op_b;
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
                        // Post-mul: accumulate env*sbf into the output reg.
                        // 8-bit modular addition; downstream truncates to
                        // 8 bits anyway. The last accumulation (filt_idx==8)
                        // also pulses done and returns to idle in one cycle.
                        out <= out + prod_trunc_envsaw;
                        if (filt_idx == 4'd8) begin
                            done      <= 1'b1;
                            busy      <= 1'b0;
                            filt_idx  <= 4'd0;
                            mac_phase <= 3'd0;
                        end else begin
                            filt_idx  <= filt_idx + 4'd1;
                            mac_phase <= 3'd0;
                        end
                    end else begin
                        // MAC phase 0..3 -- capture product and update state.
                        // Envelope filters (is_env) skip phases 1 and 3:
                        // b2 and a2 are zero by topology for a one-pole IIR,
                        // so the matching multiplies and the s2 update would
                        // produce nothing. The slot[] capture that BPFs do
                        // at phase 3 happens at phase 2 instead for env.
                        case (mac_phase)
                            // Phase 0 (b0*x): fold the trailing `+ s1` into
                            // the same cycle so the b0 product never needs
                            // its own holding register. (b1 phase is omitted
                            // entirely -- b1 is zero in this design.)
                            3'd0: y_r   <= prod_trunc_filt + s1[filt_idx];
                            // Phase 1 (b2*x): reachable only for BPF / SBF.
                            3'd1: xb2_r <= prod_trunc_filt;
                            // Phase 2 (a1*y): write the new s1 immediately
                            // with the fresh a1 product. For env, s2 is
                            // permanently 0 so we don't read it -- and we
                            // also capture y_r into slot[] now since the
                            // env biquad ends here.
                            3'd2: begin
                                if (is_env) begin
                                    s1[filt_idx] <= -prod_trunc_filt;
                                    case (filt_idx)
                                        4'd3:    slot[0] <= y_r;
                                        4'd4:    slot[1] <= y_r;
                                        4'd5:    slot[2] <= y_r;
                                        default: ;
                                    endcase
                                end else begin
                                    s1[filt_idx] <= s2[s2_addr] - prod_trunc_filt;
                                end
                            end
                            // Phase 3 (a2*y + s2 write): reachable only for
                            // BPF (filt_idx 0..2) or SBF (filt_idx 6..8).
                            3'd3: begin
                                s2[s2_addr] <= xb2_r - prod_trunc_filt;
                                case (filt_idx)
                                    4'd0:    slot[0] <= y_r;
                                    4'd1:    slot[1] <= y_r;
                                    4'd2:    slot[2] <= y_r;
                                    // SBF (6..8): y_r goes straight into
                                    // the post-mul; no slot capture.
                                    default: ;
                                endcase
                            end
                            default: ;
                        endcase

                        // Phase transition:
                        //   BPF / SBF : 0 -> 1 -> 2 -> 3 -> {next | post-mul}
                        //   ENV       : 0 -> 2 -> next biquad (skip 1 and 3)
                        case (mac_phase)
                            3'd0:    mac_phase <= is_env ? 3'd2 : 3'd1;
                            3'd1:    mac_phase <= 3'd2;
                            3'd2: begin
                                if (is_env) begin
                                    filt_idx  <= filt_idx + 4'd1;
                                    mac_phase <= 3'd0;
                                end else begin
                                    mac_phase <= 3'd3;
                                end
                            end
                            3'd3: begin
                                if (filt_idx >= 4'd6) begin
                                    mac_phase <= 3'd4;  // SBF needs post-mul
                                end else begin
                                    filt_idx  <= filt_idx + 4'd1;
                                    mac_phase <= 3'd0;
                                end
                            end
                            default: ;
                        endcase
                    end
                    end
            end
        end
    end
endmodule
