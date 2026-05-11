`default_nettype none
`timescale 1ns / 1ps

// Testbench for the vocoder DSP block. All coefficient parameters
// are lifted to this toplevel so the cocotb runner can override them
// via `parameters=...` on Runner.build().
module tb_vocoder ();

    // ── Coefficient parameters (Q2.14 signed, defaults to 0) ────────────
    parameter signed [7:0] B1_b0 = 8'sd0;
    parameter signed [7:0] B1_b1 = 8'sd0;
    parameter signed [7:0] B1_b2 = 8'sd0;
    parameter signed [7:0] B1_a1 = 8'sd0;
    parameter signed [7:0] B1_a2 = 8'sd0;

    parameter signed [7:0] B2_b0 = 8'sd0;
    parameter signed [7:0] B2_b1 = 8'sd0;
    parameter signed [7:0] B2_b2 = 8'sd0;
    parameter signed [7:0] B2_a1 = 8'sd0;
    parameter signed [7:0] B2_a2 = 8'sd0;

    parameter signed [7:0] B3_b0 = 8'sd0;
    parameter signed [7:0] B3_b1 = 8'sd0;
    parameter signed [7:0] B3_b2 = 8'sd0;
    parameter signed [7:0] B3_a1 = 8'sd0;
    parameter signed [7:0] B3_a2 = 8'sd0;

    parameter signed [7:0] ENV_b0 = 8'sd0;
    parameter signed [7:0] ENV_a1 = 8'sd0;

    // ── DUT-facing nets (driven by cocotb) ─────────────────────────────
    reg               clk   = 1'b0;
    reg               rst_n = 1'b0;
    reg  signed [7:0] mic   = 8'sd0;
    reg  signed [7:0] saw   = 8'sd0;
    wire signed [7:0] out;

    // Vocoder uses a start/done handshake: pulse `start` for one cycle
    // with the desired mic/saw, then wait for `done` to sample `out`.
    reg  start = 1'b0;
    wire done;

`ifndef VERILATOR
    initial begin
        $dumpfile("tb_vocoder.fst");
        $dumpvars(0, tb_vocoder);
    end
`endif

    vocoder #(
        .B1_b0(B1_b0), .B1_b1(B1_b1), .B1_b2(B1_b2),
        .B1_a1(B1_a1), .B1_a2(B1_a2),
        .B2_b0(B2_b0), .B2_b1(B2_b1), .B2_b2(B2_b2),
        .B2_a1(B2_a1), .B2_a2(B2_a2),
        .B3_b0(B3_b0), .B3_b1(B3_b1), .B3_b2(B3_b2),
        .B3_a1(B3_a1), .B3_a2(B3_a2),
        .ENV_b0(ENV_b0), .ENV_a1(ENV_a1)
    ) dut (
        .clk  (clk),
        .rst_n(rst_n),
        .start(start),
        .done (done),
        .mic  (mic),
        .saw  (saw),
        .out  (out)
    );

endmodule
