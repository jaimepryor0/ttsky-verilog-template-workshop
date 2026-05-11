`default_nettype none
`timescale 1ns / 1ps

// Standalone testbench for spi_master. Exposes every port as a toplevel
// signal so cocotbext-spi.SpiBus can attach to them by name.
module tb_spi ();

    reg          clk = 1'b0;
    reg          rst_n = 1'b0;
    reg          start = 1'b0;
    reg  [15:0]  tx_data = 16'h0000;
    wire         done;
    wire [15:0]  rx_data;

    wire         cs_n;
    wire         sck;
    wire         mosi;
    reg          miso = 1'b0;

`ifndef VERILATOR
    initial begin
        $dumpfile("tb_spi.fst");
        $dumpvars(0, tb_spi);
    end
`endif

    spi_master #(.SCK_DIV(16)) dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (start),
        .tx_data(tx_data),
        .done   (done),
        .rx_data(rx_data),
        .cs_n   (cs_n),
        .sck    (sck),
        .mosi   (mosi),
        .miso   (miso)
    );

endmodule
