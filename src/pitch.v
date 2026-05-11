`default_nettype none

module pitch(clk, rst_n, en, increment, out);
// Sawtooth NCO. Output is signed Q1.7 ramping from -1.0 to ~+1.0
// across each period, then wrapping. Frequency = Fs * increment / 256
// where Fs is the audio sample rate (i.e. the rate at which `en` pulses).
//
// The full 32-bit phase accumulator in the previous revision was wasted
// silicon: the chip only drives the top 8 bits of the increment (the
// `ui_in` pitch byte), so the bottom 24 bits of the accumulator were
// permanently zero. We collapse the accumulator to the 8 bits that
// actually move.

    input  wire              clk;
    input  wire              rst_n;
    input  wire              en;          // 1-cycle pulse per audio sample
    input  wire        [7:0] increment;   // ui_in pitch byte
    output wire signed [7:0] out;

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

    // Reinterpret as Q1.7 signed -> bipolar sawtooth.
    assign out = $signed(phase);

endmodule
