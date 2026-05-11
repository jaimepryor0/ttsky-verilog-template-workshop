`default_nettype none

module pitch(clk, rst_n, en, increment, out);
// Sawtooth NCO. Output is signed Q1.7 ramping from -1.0 to ~+1.0
// across each period, then wrapping. Frequency = Fs * increment / 2^32
// where Fs is the audio sample rate (i.e. the rate at which `en` pulses).

    parameter PHASE_W = 32;

    input  wire                       clk;
    input  wire                       rst_n;
    input  wire                       en;            // 1-cycle pulse per audio sample
    input  wire        [PHASE_W-1:0]  increment;
    output wire signed [7:0]          out;

    reg [PHASE_W-1:0] phase;

    always @(posedge clk) begin
        if (~rst_n)
            // Start at the unsigned mid-point so that, reinterpreted as
            // signed, the first sample is -1.0 — same convention scipy's
            // sawtooth() uses at t=0.
            phase <= {1'b1, {(PHASE_W-1){1'b0}}};
        else if (en)
            phase <= phase + increment;
    end

    // Top 8 bits of the phase accumulator, reinterpreted as a signed Q1.7
    // value, give a bipolar sawtooth.
    assign out = $signed(phase[PHASE_W-1 : PHASE_W-8]);

endmodule
