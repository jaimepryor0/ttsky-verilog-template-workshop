`default_nettype none

module pitch(clk, rst_n, en, increment, mode, out);
// Multi-waveform NCO. Output is signed Q1.7. Frequency = Fs * increment / 256
// where Fs is the audio sample rate (i.e. the rate at which `en` pulses).
//
// The mode input selects between four bipolar waveforms derived from the
// same phase counter:
//   2'b00 -> sawtooth (rising)
//   2'b01 -> sawtooth (falling) — bit-inverse of rising saw
//   2'b10 -> square (half amplitude: ±64) — abrupt level switch on phase[7]
//   2'b11 -> triangle (half amplitude: -64..+63) — `|saw| - 64`
//
// Square and triangle are amplitude-halved so the abs/negate stages can't
// overflow the 8-bit signed range when phase = -128. The vocoder treats
// these the same as any other carrier sample.
//
// The 8-bit accumulator is unchanged from the saw-only version: the
// chip's pitch input is at most 8 bits wide, and the original 32-bit
// accumulator's bottom 24 bits never moved.

    input  wire              clk;
    input  wire              rst_n;
    input  wire              en;          // 1-cycle pulse per audio sample
    input  wire        [7:0] increment;   // phase step per sample
    input  wire        [1:0] mode;        // waveform select
    output reg  signed [7:0] out;

    reg [7:0] phase;

    always @(posedge clk) begin
        if (~rst_n)
            // Start at the unsigned mid-point so that, reinterpreted as
            // signed, the first sample is -1.0 -- same convention scipy's
            // sawtooth() uses at t=0.
            phase <= 8'h80;
        else if (en)
            phase <= phase + increment;
    end

    // Signed reinterpretation of the phase accumulator (the saw output).
    wire signed [7:0] saw_signed = $signed(phase);

    // |saw| -- magnitude of the sawtooth, 0..128 (the 128 case is the
    // -128 sample, whose negation wraps to itself).
    wire signed [7:0] abs_saw = saw_signed[7] ? -saw_signed : saw_signed;

    always @* begin
        case (mode)
            2'b00:   out = saw_signed;                        // saw up
            2'b01:   out = ~saw_signed;                       // saw down (bit-invert)
            2'b10:   out = saw_signed[7] ? -8'sd64 : 8'sd63;  // square (half-amp)
            2'b11:   out = abs_saw - 8'sd64;                  // triangle (half-amp)
            default: out = saw_signed;
        endcase
    end

endmodule
