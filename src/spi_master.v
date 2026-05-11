`default_nettype none

module spi_master(clk, rst_n, start, tx_data, done, rx_data,
                  cs_n, sck, mosi, miso);
// 16-bit SPI Mode 0 master.
//
//   Mode 0  →  SCK idles low, MOSI changes on falling edge,
//              MISO sampled on rising edge. MSB first.
//
// One transaction = `start` strobed for 1 clk → 16 SCK pulses → `done`
// strobed for 1 clk. CS is asserted low for the duration and deasserted
// at the end. The external user can mux this single CS to any number of
// slaves.

    parameter SCK_DIV  = 16;            // chip clk cycles per full SCK period
    localparam CNT_W   = $clog2(SCK_DIV);   // exactly wide enough for 0..SCK_DIV-1
    // Compare targets sized to CNT_W so synthesis doesn't generate a 32-bit
    // comparator. SCK_DIV is small enough that the truncation is a no-op.
    /* verilator lint_off WIDTHTRUNC */
    localparam [CNT_W-1:0] HALF_MAX = (SCK_DIV / 2) - 1;
    localparam [CNT_W-1:0] FULL_MAX = SCK_DIV - 1;
    /* verilator lint_on WIDTHTRUNC */

    input  wire        clk;
    input  wire        rst_n;
    input  wire        start;           // 1-cycle pulse begins a transaction
    input  wire [15:0] tx_data;         // MSB shifted out first
    output reg         done;            // 1-cycle pulse when rx_data is valid
    output reg  [15:0] rx_data;         // MSB = first bit received

    output reg         cs_n;
    output reg         sck;
    output reg         mosi;
    input  wire        miso;

    localparam S_IDLE   = 2'd0;
    localparam S_ACTIVE = 2'd1;
    localparam S_FINISH = 2'd2;

    reg [1:0] state;
    /* verilator lint_off UNUSEDSIGNAL */
    reg [15:0]      shift;              // shift[15] is read once via MOSI then dropped
    /* verilator lint_on UNUSEDSIGNAL */
    reg [3:0]       bit_idx;            // counts 0..15 across the 16-bit packet
    reg [CNT_W-1:0] cnt;                // sub-SCK counter, 0..SCK_DIV-1

    always @(posedge clk) begin
        if (~rst_n) begin
            state    <= S_IDLE;
            cs_n     <= 1'b1;
            sck      <= 1'b0;
            mosi     <= 1'b0;
            done     <= 1'b0;
            rx_data  <= 16'h0;
            shift    <= 16'h0;
            bit_idx  <= 4'd0;
            cnt      <= {CNT_W{1'b0}};
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    sck  <= 1'b0;
                    cs_n <= 1'b1;
                    if (start) begin
                        shift   <= tx_data;
                        mosi    <= tx_data[15];   // first bit out before SCK rises
                        bit_idx <= 4'd0;
                        cnt     <= {CNT_W{1'b0}};
                        cs_n    <= 1'b0;
                        state   <= S_ACTIVE;
                    end
                end

                S_ACTIVE: begin
                    cnt <= cnt + 1'b1;
                    if (cnt == HALF_MAX) begin
                        // Rising edge of SCK — slave latches MOSI, master samples MISO.
                        sck     <= 1'b1;
                        rx_data <= {rx_data[14:0], miso};
                    end else if (cnt == FULL_MAX) begin
                        // Falling edge of SCK — shift next bit out.
                        sck <= 1'b0;
                        cnt <= {CNT_W{1'b0}};
                        if (bit_idx == 4'd15) begin
                            state <= S_FINISH;
                        end else begin
                            bit_idx <= bit_idx + 4'd1;
                            shift   <= {shift[14:0], 1'b0};
                            mosi    <= shift[14];
                        end
                    end
                end

                S_FINISH: begin
                    cs_n  <= 1'b1;
                    sck   <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
