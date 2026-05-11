/*
 * Channel vocoder for TinyTapeout -- parallel audio interface.
 *
 * Pipeline per audio sample:
 *   1. Producer drives ui_in[6:0] with a new Q1.6 sample.
 *   2. Producer rising-edges ui_in[7]; that pulse starts the vocoder
 *      and advances the sawtooth NCO by one sample.
 *   3. Vocoder runs (~350 chip clocks) on the latched mic + saw values.
 *   4. uo_out[6:0] holds the new Q1.6 result; uo_out[7] pulses high for
 *      one chip clock to mark it.
 *
 * Pinout:
 *   ui_in[6:0]    audio input sample (7-bit signed Q1.6)
 *   ui_in[7]      input valid (rising edge latches the sample + starts a cycle)
 *   uo_out[6:0]   audio output sample (7-bit signed Q1.6; top 7 bits of the
 *                 vocoder's 8-bit Q1.7 result, saturating-clipped)
 *   uo_out[7]     output valid (LEVEL: drops to 0 when a new start is seen,
 *                 rises to 1 when the new sample is registered; stays high
 *                 between samples. Async consumers can poll or edge-trigger.)
 *   uio_in[7:6]   carrier waveform select (0=saw, 1=inv saw, 2=square, 3=tri)
 *   uio_in[5:0]   pitch byte (6-bit; mapped to the LOW 6 bits of an 8-bit NCO
 *                 increment, giving f = pitch * Fs / 256 -- about 187 Hz per
 *                 step at Fs=48k. Range 187 Hz .. 11.8 kHz, covers voice
 *                 fundamentals comfortably.)
 *   uio_out / uio_oe   driven low (uio used as input only)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

/* verilator lint_off DECLFILENAME */
module tt_um_JAIMEPRYOR0_VGA_YAY(
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // All bidirectional pins are inputs (pitch byte).
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // ena is held high by the harness; not used internally.
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused = &{ena, 1'b0};
    /* verilator lint_on UNUSEDSIGNAL */

    // --- Rising-edge detector on ui_in[7] ---------------------------------
    // The producer is expected to lower ui_in[7] between samples so each
    // new sample comes in as a fresh 0->1 transition. We pulse start
    // for one cycle on that edge; the vocoder ignores further starts
    // while it's busy, so spurious edges during processing are dropped.
    reg  valid_in_d;
    wire start_pulse = ui_in[7] & ~valid_in_d;
    always @(posedge clk) begin
        if (~rst_n) valid_in_d <= 1'b0;
        else        valid_in_d <= ui_in[7];
    end

    // --- Sample-rate datapath --------------------------------------------
    // External pin sample is 7-bit signed (Q1.6); left-shift by 1 to feed
    // the vocoder's 8-bit Q1.7 mic input. The LSB lands at 0 -- a one-bit
    // precision loss we accept in exchange for the 7+1 packing.
    wire signed [7:0] mic_q7 = {ui_in[6:0], 1'b0};
    wire signed [7:0] saw_q7;
    // vocoder_out[0] is the discarded LSB of the 8-bit Q1.7 result; only
    // the top 7 bits land on the pin.
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [7:0] vocoder_out;
    /* verilator lint_on UNUSEDSIGNAL */
    wire              vocoder_done;

    // Pitch byte is the bottom 6 bits of uio_in, zero-padded into the low
    // 6 bits of an 8-bit increment. That puts the smallest step at the
    // bottom of the frequency range (Fs/256 ~ 187 Hz at 48 kHz) where it
    // matters for voice carriers. Max output ~Fs * 63/256 ~ 11.8 kHz.
    pitch u_pitch (
        .clk      (clk),
        .rst_n    (rst_n),
        .en       (start_pulse),
        .increment({2'b00, uio_in[5:0]}),
        .mode     (uio_in[7:6]),
        .out      (saw_q7)
    );

    // Coefficients (Q2.6) generated from vocoder_fixed_point.VOICE_BANDS at FS=24k.
    // Formant-band design (B1: 800-2500 Hz F2, B2: 2500-5000 Hz F3). b1 is
    // not a parameter -- vocoder.v skips that multiply phase entirely because
    // every band's b1 quantises to zero at this filter design.
    // ENV: 300 Hz cutoff at FS=24k -> (1-alpha)=5/64, alpha=59/64.
    vocoder #(
        .B1_b0( 8'sd12 ), .B1_b2(-8'sd12),
        .B1_a1(-8'sd97 ), .B1_a2( 8'sd40),
        .B2_b0( 8'sd16 ), .B2_b2(-8'sd16),
        .B2_a1(-8'sd56 ), .B2_a2( 8'sd32),
        .ENV_b0(8'sd5),   .ENV_a1(-8'sd59)
    ) u_vocoder (
        .clk  (clk),
        .rst_n(rst_n),
        .start(start_pulse),
        .done (vocoder_done),
        .mic  (mic_q7),
        .saw  (saw_q7),
        .out  (vocoder_out)
    );

    // --- Outputs ----------------------------------------------------------
    // Drop the LSB of the 8-bit Q1.7 result for a 7-bit Q1.6 pin output;
    // vocoder_out holds its value between done pulses, so consumers can
    // sample uo_out[6:0] on the rising edge of uo_out[7].
    assign uo_out[6:0] = vocoder_out[7:1];
    assign uo_out[7]   = vocoder_done;

endmodule
