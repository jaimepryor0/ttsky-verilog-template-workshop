
module vocoder(clk, rst_n, mic, out, saw_period);
//Todo: add ability to vary sawtooth period 
    input clk; 
    input rst_n; // rst_n is negative triggered (active low) reset. Preferred with ASICs because when powering on, everything is zero, 
    // which here will mean reset 
    input [7:0] mic;
    output [7:0] out;

    // FILTER COEFFICIENTS //
    localparam B1_a0 8'b00000001 
    localparam B1_a1 8'b00000010 // a1 for first bandpass filter

    ///////////////////////////////////////

    // always blocks: whenever something in the sensitivity list changes, the stuff
    // in the block will be run sequentially 

    wire [7:0] mic; 

    // Define wires to receive outputs from bandpass filters
    wire [7:0] bf1_out;
    wire [7:0] bf2_out;
    wire [7:0] bf3_out;

    // <module name> <instantiation name>(.<portname>(passval), );
    // module for bandpass filter 
    filter bf1(.clk(clk), .rst(rst), .b0(), .b1(), .b2(), .a0(B1_a0), .a1(B1_a1), .a2(), .in(mic), .out(bf1_out));
    filter bf2(.clk(clk), .rst(rst), .b0(), .b1(), .b2(), .a0(), .a1(), .a2(), .in(mic), .out(bf2_out));
    filter bf3(.clk(clk), .rst(rst), .b0(), .b1(), .b2(), .a0(), .a1(), .a2(), .in(mic), .out(bf3_out));

    // Define wires as ouptuts from envelope 
    wire [7:0] e1_out;
    wire [7:0] e2_out;
    wire [7:0] e3_out;

    // TODO: rectify by flipping bits and adding 1 (absolute value)

    // Envelope extraction 
    filter e1_out(.clk(clk), .rst(rst), .b0(), .b1(), .b2(), .a0(B1_a0), .a1(B1_a1), .a2(), .in(mic), .out(bf1_out));
    filter 
    filter 

    // TODO: implement envelope extraction with single filter

    // Define wire for pitch signal 
    wire [7:0] pitch_sig; 

    // TODO: implement sawtooth generator 

    // Pitch signal generation 
    pitch pitch(.clk(clk), .rst(rst), .out(pitch_sig))

    wire [7:0] mult1_out; 
    wire [7:0] mult2_out; 
    wire [7:0] mult3_out; 

    // Define outputs for multipliers 
    multiplier mult1(.clk(clk), .rst(rst), .in1(pitch_sig), .in2(e1_out), .out(mult1_out))
    multiplier mult2(.clk(clk), .rst(rst), .in1(pitch_sig), .in2(e2_out), .out(mult2_out))
    multiplier mult3(.clk(clk), .rst(rst), .in1(pitch_sig), .in2(e3_out), .out(mult3_out))

    wire [7:0] adder_out; 

    adder add(.clk(clk), .rst(rst), .in1(mult1_out), .in2(mult2_out), .in3(mult3_out), .out(adder_out))

    // Assign output
    assign out = adder_out


endmodule 



