`default_nettype none

module vocoder(clk, rst_n, start, done, mic, saw, out);
// Two-band channel vocoder, matching vocoder_fixed_point.py exactly.
//
//   mic -> 2 bandpass filters -> |.| -> envelope LPF -.
//                                                      \
//                                                       > x -> sum -> out
//                                                      /
//   saw -> the same 2 bandpass filters ----------------'
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
//   * b2 and a2 are zero for the envelope LPFs (filt_idx 2..3) by
//     topology -- a one-pole IIR has no second-order taps. The b2*x and
//     a2*y phases are skipped for those two slots, so each envelope
//     biquad runs only 2 multiplies (b0*x, a1*y) instead of 4. s2[2..3]
//     is never written and gets pruned by synthesis.
//
// Each multiply takes 1 setup cycle + 8 iteration cycles = 9 chip clocks.
// Total per audio sample:
//   (2 BPF * 4 phases + 2 ENV * 2 phases + 2 SBF * 5 phases) * 9 + 1 = 199 cycles.
//
// Interface:
//   start - 1-cycle pulse to begin processing one sample using the current
//           mic and saw inputs (latched internally on the pulse).
//   done  - LEVEL signal. Goes low on `start` (output is stale, recomputing)
//           and back high on the cycle the new sample is registered into
//           `out`. Stays high until the next start. Consumers can either
//           watch for the 0->1 transition or just poll the level.
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

    // Envelope LPF: one-pole IIR, b = [1-alpha, 0], a = [1, -alpha]
    parameter signed [7:0] ENV_b0 = 8'sd0;
    parameter signed [7:0] ENV_a1 = 8'sd0;

    ///////////////////////////////////////
    // Filter slot indices (3-bit filt_idx)
    //   0..1 = mic bandpass (B1, B2)
    //   2..3 = envelope LPF (E1, E2) reading rectified mic-BPF outputs
    //   4..5 = saw bandpass (B1, B2) -- same coeffs as 0..1
    //
    // Bit layout:
    //   filt_idx[2] = 1 iff this is the saw-side BPF (post-mul required)
    //   filt_idx[1] = 1 iff this is the envelope LPF (single-pole, skip b2/a2)
    //   filt_idx[0] = band index within the slot (0 = B1, 1 = B2)
    ///////////////////////////////////////

    // --- Latched inputs (captured when start is sampled high) ---
    reg signed [7:0] mic_r, saw_r;

    // --- State RAM ---
    // s1 has one entry per biquad slot, including the envelope filters.
    // s2 is only used by the second-order biquads (2 BPF + 2 SBF = 4 slots);
    // the envelope filters are one-pole so their s2 would be permanently
    // zero. We address s2 with the compressed s2_addr below.
    reg signed [7:0] s1 [0:5];
    reg signed [7:0] s2 [0:3];

    // --- Shared per-band output slot --------------------------------------
    // For band i, slot[i] holds:
    //   - the mic BPF result (bf_y[i]) from end of biquad i until end of
    //     biquad i+2, where the envelope filter consumes it via `rect`;
    //   - the envelope LPF result (env_y[i]) from end of biquad i+2 until
    //     phase 4 of biquad i+4, where the SBF post-multiply reads it.
    // The two live ranges are disjoint, so a single 2-entry array carries
    // both.
    reg signed [7:0] slot [0:1];

    // --- Per-filter MAC intermediates ---
    // Phase 0's product is folded into `y_r` directly (no xb0_r). Phase 1
    // captures the b2 product into xb2_r. The a1 product is consumed in
    // the same cycle it's produced (phase 2) by writing s1 there.
    reg signed [7:0] xb2_r;
    reg signed [7:0] y_r;

    // --- Control ---
    reg [2:0] filt_idx;   // 0..5
    reg [2:0] mac_phase;  // 0..4 (0..3 = MAC, 4 = post-mul / final accumulate)
    reg [3:0] sub_cnt;    // 0..8: 0 = operand setup, 1..8 = serial-mult iterations
    reg       busy;

    // The output register doubles as the per-sample post-mul accumulator: it
    // gets cleared to 0 on `start`, accumulates env*sbf products at phase 4,
    // and is left holding the new sample the moment `done` pulses.

    // --- Serial signed multiplier state ---
    reg signed [7:0] m_acc;
    reg        [7:0] m_b;

    // Rectify (abs) of mic-BPF outputs -- feeds envelope LPF inputs.
    wire signed [7:0] rect0 = slot[0][7] ? -slot[0] : slot[0];
    wire signed [7:0] rect1 = slot[1][7] ? -slot[1] : slot[1];

    // is_env: filt_idx is 2 or 3 (single-pole envelope LPF).
    // is_sbf: filt_idx is 4 or 5 (saw bandpass, needs post-mul).
    wire is_env = (filt_idx[2:1] == 2'b01);
    wire is_sbf =  filt_idx[2];

    // Compress filt_idx {0,1,4,5} -> s2 address {0,1,2,3}. filt_idx[1] is
    // 0 for BPF/SBF so {filt_idx[2], filt_idx[0]} gives the right mapping.
    // For envelopes (where filt_idx[1]=1) we don't touch s2, so the value
    // doesn't matter.
    wire [1:0] s2_addr = {filt_idx[2], filt_idx[0]};

    // --- Coefficient ROM (combinational mux on filt_idx) ---
    reg signed [7:0] b0_sel, b2_sel, a1_sel, a2_sel;
    always @* begin
        case (filt_idx)
            3'd0, 3'd4: begin b0_sel=B1_b0; b2_sel=B1_b2; a1_sel=B1_a1; a2_sel=B1_a2; end
            3'd1, 3'd5: begin b0_sel=B2_b0; b2_sel=B2_b2; a1_sel=B2_a1; a2_sel=B2_a2; end
            3'd2, 3'd3: begin b0_sel=ENV_b0; b2_sel=8'sd0; a1_sel=ENV_a1; a2_sel=8'sd0; end
            default:    begin b0_sel=8'sd0; b2_sel=8'sd0; a1_sel=8'sd0; a2_sel=8'sd0; end
        endcase
    end

    // --- Filter input mux ---
    reg signed [7:0] in_sel;
    always @* begin
        case (filt_idx)
            3'd0, 3'd1: in_sel = mic_r;
            3'd2:       in_sel = rect0;
            3'd3:       in_sel = rect1;
            3'd4, 3'd5: in_sel = saw_r;
            default:    in_sel = 8'sd0;
        endcase
    end

    // --- Operand selection per phase ---
    //   phase 0 : in_sel * b0     -- folds in `+ s1` to make y_r
    //   phase 1 : in_sel * b2     -- skipped for env
    //   phase 2 : y_r    * a1     -- s1 updated this cycle
    //   phase 3 : y_r    * a2     -- s2 updated; skipped for env
    //   phase 4 : slot[filt_idx[0]] * y_r  -- SBF post-mul
    reg signed [7:0] op_a, op_b;
    always @* begin
        case (mac_phase)
            3'd0: begin op_a = in_sel; op_b = b0_sel; end
            3'd1: begin op_a = in_sel; op_b = b2_sel; end
            3'd2: begin op_a = y_r;    op_b = a1_sel; end
            3'd3: begin op_a = y_r;    op_b = a2_sel; end
            3'd4: begin
                case (filt_idx[0])
                    1'b0:    begin op_a = slot[0]; op_b = y_r; end
                    1'b1:    begin op_a = slot[1]; op_b = y_r; end
                    default: begin op_a = 8'sd0;   op_b = 8'sd0; end
                endcase
            end
            default: begin op_a = 8'sd0; op_b = 8'sd0; end
        endcase
    end

    // --- Serial multiplier iteration (combinational) ---
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

    wire signed [7:0] m_acc_after = m_new_top[8:1];
    wire        [7:0] m_b_after   = {m_new_top[0], m_b[7:1]};

    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [15:0] mul_prod = {m_acc_after, m_b_after};
    /* verilator lint_on UNUSEDSIGNAL */
    wire signed [7:0] prod_trunc_filt   = mul_prod[13:6];
    wire signed [7:0] prod_trunc_envsaw = mul_prod[14:7];

    // Saturating accumulator for the env*sbf products. The previous
    // 8-bit modular add wrapped on overflow, producing audible "clicks"
    // when the band products summed past +-1.0. Clamp to the Q1.7 range.
    wire signed [8:0] out_sum     = {out[7], out} + {prod_trunc_envsaw[7], prod_trunc_envsaw};
    wire signed [7:0] out_clipped = (out_sum >  9'sd127)  ?  8'sd127 :
                                    (out_sum < -9'sd128)  ? -8'sd128 : out_sum[7:0];

    // --- FSM ---
    always @(posedge clk) begin
        if (~rst_n) begin
            filt_idx  <= 3'd0;
            mac_phase <= 3'd0;
            sub_cnt   <= 4'd0;
            busy      <= 1'b0;
            done      <= 1'b0;
            out       <= 8'sd0;
            m_acc     <= 8'sd0;
            m_b       <= 8'd0;
            // NOTE: s1/s2/slot/xb2_r/y_r/mic_r/saw_r intentionally have NO
            // reset (DFXTP, not DFRTP). The first ~50 audio samples after
            // power-up are transient noise as the IIR filters wash the
            // power-on state out -- inaudible on an audio path. The
            // cocotb regression skips that window when comparing against
            // the Python reference (see WARMUP in test_chip / test_vocoder).
        end else begin
            // done has level semantics: it holds its value unless a new
            // start arrives (clear) or a sample finishes (set).
            if (!busy) begin
                if (start) begin
                    busy      <= 1'b1;
                    done      <= 1'b0;      // output is now stale, recomputing
                    filt_idx  <= 3'd0;
                    mac_phase <= 3'd0;
                    sub_cnt   <= 4'd0;
                    mic_r     <= mic;
                    saw_r     <= saw;
                    out       <= 8'sd0;     // also the post-mul accumulator
                end
            end else begin
                if (sub_cnt == 4'd0) begin
                    m_acc   <= 8'sd0;
                    m_b     <= op_b;
                    sub_cnt <= 4'd1;
                end else if (sub_cnt != 4'd8) begin
                    m_acc   <= m_acc_after;
                    m_b     <= m_b_after;
                    sub_cnt <= sub_cnt + 4'd1;
                end else begin
                    m_acc   <= m_acc_after;
                    m_b     <= m_b_after;
                    sub_cnt <= 4'd0;

                    if (mac_phase == 3'd4) begin
                        // SBF post-mul: accumulate env*sbf into output reg
                        // with saturating clip (Q1.7 range).
                        out <= out_clipped;
                        if (filt_idx == 3'd5) begin
                            done      <= 1'b1;
                            busy      <= 1'b0;
                            filt_idx  <= 3'd0;
                            mac_phase <= 3'd0;
                        end else begin
                            filt_idx  <= filt_idx + 3'd1;
                            mac_phase <= 3'd0;
                        end
                    end else begin
                        // MAC phase 0..3 -- capture product and update state.
                        case (mac_phase)
                            3'd0: y_r   <= prod_trunc_filt + s1[filt_idx];
                            3'd1: xb2_r <= prod_trunc_filt;
                            3'd2: begin
                                if (is_env) begin
                                    s1[filt_idx] <= -prod_trunc_filt;
                                    case (filt_idx[0])
                                        1'b0:    slot[0] <= y_r;
                                        1'b1:    slot[1] <= y_r;
                                        default: ;
                                    endcase
                                end else begin
                                    s1[filt_idx] <= s2[s2_addr] - prod_trunc_filt;
                                end
                            end
                            3'd3: begin
                                s2[s2_addr] <= xb2_r - prod_trunc_filt;
                                if (!is_sbf) begin
                                    // mic BPF: capture y_r to slot for the
                                    // matching envelope filter to consume.
                                    case (filt_idx[0])
                                        1'b0:    slot[0] <= y_r;
                                        1'b1:    slot[1] <= y_r;
                                        default: ;
                                    endcase
                                end
                                // SBF: y_r goes straight into the post-mul,
                                // no slot capture.
                            end
                            default: ;
                        endcase

                        case (mac_phase)
                            3'd0:    mac_phase <= is_env ? 3'd2 : 3'd1;
                            3'd1:    mac_phase <= 3'd2;
                            3'd2: begin
                                if (is_env) begin
                                    filt_idx  <= filt_idx + 3'd1;
                                    mac_phase <= 3'd0;
                                end else begin
                                    mac_phase <= 3'd3;
                                end
                            end
                            3'd3: begin
                                if (is_sbf) begin
                                    mac_phase <= 3'd4;   // SBF post-mul
                                end else begin
                                    filt_idx  <= filt_idx + 3'd1;
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
