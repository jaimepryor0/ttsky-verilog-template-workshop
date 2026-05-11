`default_nettype none
`timescale 1ns / 1ps

// Chip-level testbench for the parallel-audio top.
// cocotb drives ui_in (7-bit sample + valid bit) and uio_in (pitch byte),
// then reads uo_out[6:0] each time uo_out[7] rises.
module tb_project ();

    reg        clk    = 1'b0;
    reg        rst_n  = 1'b0;
    reg        ena    = 1'b1;
    reg  [7:0] ui_in  = 8'd0;
    reg  [7:0] uio_in = 8'd0;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // Promote the output-valid bit so cocotb's RisingEdge() can latch on it.
    wire out_valid = uo_out[7];

`ifndef VERILATOR
    initial begin
        $dumpfile("tb_project.fst");
        $dumpvars(0, tb_project);
    end
`endif

    tt_um_JAIMEPRYOR0_VGA_YAY dut (
        .ui_in  (ui_in),
        .uo_out (uo_out),
        .uio_in (uio_in),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (ena),
        .clk    (clk),
        .rst_n  (rst_n)
    );

endmodule
