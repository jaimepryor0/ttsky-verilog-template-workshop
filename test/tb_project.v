`default_nettype none
`timescale 1ns / 1ps

// Chip-level testbench. Instantiates the full tt_um_* wrapper and exposes
// individual SPI pins as named wires so cocotb-side mock slaves can edge-trigger
// on them directly.
module tb_project ();

    reg        clk         = 1'b0;
    reg        rst_n       = 1'b0;
    reg        ena         = 1'b1;
    reg [7:0]  ui_in       = 8'd0;
    reg        miso_drive  = 1'b0;

    // The chip configures uio[2] as the only input (MISO). Drive that bit
    // from the cocotb mock; everything else on uio_in is don't-care.
    wire [7:0] uio_in;
    assign uio_in[7:3] = 5'b0;
    assign uio_in[2]   = miso_drive;
    assign uio_in[1:0] = 2'b0;

    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // Convenience taps. Cocotb's RisingEdge / FallingEdge only operate on a
    // single 1-bit signal, so we promote the bits we care about.
    wire adc_cs = uio_out[0];
    wire mosi   = uio_out[1];
    wire sck    = uio_out[3];
    wire dac_cs = uio_out[4];

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
