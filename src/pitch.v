module pitch(clk, rst_n, increment, out);
// Sawtooth NCO — frequency = (fs * increment) / 2^24

    input              clk, rst_n;
    input       [23:0] increment;
    output      [7:0]  out;

    reg [23:0] phase;

    always @(posedge clk) begin
        if (~rst_n)
            phase <= 0;
        else
            phase <= phase + increment;
    end

    assign out = phase[23:16];

endmodule
