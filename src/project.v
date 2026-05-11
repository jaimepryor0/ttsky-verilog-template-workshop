/*
 * Channel vocoder for TinyTapeout.
 *
 * Pipeline per audio sample:
 *   1. Read 16-bit SPI transaction from MCP3201 ADC (uio[0] = CS, uio[2] = MISO)
 *   2. Convert ADC's 12-bit unsigned reading to 16-bit signed Q1.15
 *   3. Pulse `en` to advance the vocoder + sawtooth NCO by one sample
 *   4. Capture vocoder.out into a holding register
 *   5. Write 16-bit SPI transaction to MCP4921 DAC (uio[4] = CS, uio[1] = MOSI)
 *
 * SPI is shared Mode 0, MSB-first, with separate CS lines per slave.
 *
 *   ui_in[7:0]   pitch byte: top 8 bits of the 32-bit NCO phase increment
 *   uio[0]       ADC_CS_n  (out)
 *   uio[1]       SPI MOSI  (out, drives DAC SDI)
 *   uio[2]       SPI MISO  (in,  reads ADC SDO)
 *   uio[3]       SPI SCK   (out)
 *   uio[4]       DAC_CS_n  (out)
 *   uio[7:5]     unused    (out, tied low)
 *   uo_out[7]    1-bit sigma-delta audio (drives Mike's Audio PMOD if no DAC)
 *   uo_out[6:0]  debug taps (controller state, SPI CS/SCK)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

/* verilator lint_off DECLFILENAME */
module tt_um_JAIMEPRYOR0_VGA_YAY(
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // ─── Pin breakout ───────────────────────────────────────────────────────
    wire adc_cs_n, dac_cs_n, sck, mosi;
    wire miso = uio_in[2];

    assign uio_out[0]   = adc_cs_n;
    assign uio_out[1]   = mosi;
    assign uio_out[2]   = 1'b0;             // MISO is input, drive 0 when oe=0
    assign uio_out[3]   = sck;
    assign uio_out[4]   = dac_cs_n;
    assign uio_out[7:5] = 3'b000;
    assign uio_oe       = 8'b1111_1011;     // all out except bit 2 (MISO)

    // Quieten unused-input warnings.
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused = &{ena, uio_in[7:3], uio_in[1:0], 1'b0};
    /* verilator lint_on UNUSEDSIGNAL */

    // ─── Shared SPI master ──────────────────────────────────────────────────
    reg         spi_start;
    reg  [15:0] spi_tx;
    wire        spi_done;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [15:0] spi_rx;                 // only bits [13:2] carry MCP3201 data
    /* verilator lint_on UNUSEDSIGNAL */
    wire        spi_cs_n;

    spi_master #(.SCK_DIV(16)) u_spi (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (spi_start),
        .tx_data(spi_tx),
        .done   (spi_done),
        .rx_data(spi_rx),
        .cs_n   (spi_cs_n),
        .sck    (sck),
        .mosi   (mosi),
        .miso   (miso)
    );

    // ─── Sawtooth NCO + vocoder (clock-enabled at audio sample rate) ────────
    reg  signed [12:0] mic_q12;             // latched ADC reading in Q1.12
    reg  signed [12:0] dac_q12;             // captured vocoder out for DAC
    reg                sample_en;           // 1-cycle start pulse per sample
    wire signed [12:0] saw_q12;
    wire signed [12:0] vocoder_out;
    wire               vocoder_done;        // pulses when vocoder_out is fresh

    pitch u_pitch (
        .clk      (clk),
        .rst_n    (rst_n),
        .en       (sample_en),
        .increment({ui_in, 24'b0}),         // ui_in maps to top 8 bits of 32
        .out      (saw_q12)
    );

    // Coefficients (Q2.11) generated from vocoder_fixed_point.VOICE_BANDS at FS=48k.
    // Re-run test/test_vocoder._design_parameters() if those values change.
    vocoder #(
        .B1_b0( 13'sd102 ), .B1_b1( 13'sd0   ), .B1_b2(-13'sd102 ),
        .B1_a1(-13'sd3885), .B1_a2( 13'sd1844),
        .B2_b0( 13'sd184 ), .B2_b1( 13'sd0   ), .B2_b2(-13'sd184 ),
        .B2_a1(-13'sd3697), .B2_a2( 13'sd1681),
        .B3_b0( 13'sd433 ), .B3_b1( 13'sd0   ), .B3_b2(-13'sd433 ),
        .B3_a1(-13'sd2896), .B3_a2( 13'sd1182),
        .ENV_b0(13'sd5),    .ENV_a1(-13'sd2043)
    ) u_vocoder (
        .clk  (clk),
        .rst_n(rst_n),
        .start(sample_en),
        .done (vocoder_done),
        .mic  (mic_q12),
        .saw  (saw_q12),
        .out  (vocoder_out)
    );

    // ─── Sample format conversions ──────────────────────────────────────────
    // MCP3201: 16 SCK with the 12 data bits in spi_rx[13:2] (null bit at [14],
    // sampling/trailing bits elsewhere). Bias is around 2048 = 0V. Promote to
    // 13-bit signed Q1.12 by toggling the MSB and shifting left by 1 so that
    // the 12 ADC bits sit in the top 12 bits of Q1.12 (LSB held at 0).
    wire [11:0] adc_unsigned = spi_rx[13:2];
    wire signed [12:0] adc_q12 = {~adc_unsigned[11], adc_unsigned[10:0], 1'b0};

    // MCP4921 write word: {A/B, BUF, ~GA, ~SHDN, D11..D0}. Channel A,
    // unbuffered, 1× gain, active = 4'b0011. Convert 13-bit Q1.12 to 12-bit
    // unsigned offset binary by toggling the sign bit and dropping the LSB.
    wire [11:0] dac_data    = {~dac_q12[12], dac_q12[11:1]};
    wire [15:0] dac_tx_word = {4'b0011, dac_data};

    // ─── Controller FSM ─────────────────────────────────────────────────────
    //   IDLE → ADC_WAIT → STEP → VOC_WAIT → DAC_REQ → DAC_WAIT → IDLE
    // STEP pulses sample_en for one cycle, kicking off the serial vocoder
    // (and advancing the pitch NCO). VOC_WAIT blocks until vocoder_done
    // pulses, at which point dac_q15 latches the freshly-computed sample.
    localparam S_IDLE     = 3'd0;
    localparam S_ADC_WAIT = 3'd1;
    localparam S_STEP     = 3'd2;
    localparam S_VOC_WAIT = 3'd3;
    localparam S_DAC_REQ  = 3'd4;
    localparam S_DAC_WAIT = 3'd5;

    reg [2:0] state;
    reg       target_dac;                  // routes shared CS to the right slave

    always @(posedge clk) begin
        if (~rst_n) begin
            state      <= S_IDLE;
            spi_start  <= 1'b0;
            spi_tx     <= 16'h0000;
            mic_q12    <= 13'sd0;
            dac_q12    <= 13'sd0;
            sample_en  <= 1'b0;
            target_dac <= 1'b0;
        end else begin
            spi_start <= 1'b0;             // default — pulse only when needed
            sample_en <= 1'b0;

            case (state)
                S_IDLE: begin
                    spi_tx     <= 16'h0000;
                    spi_start  <= 1'b1;    // kick off ADC read
                    target_dac <= 1'b0;
                    state      <= S_ADC_WAIT;
                end

                S_ADC_WAIT: begin
                    if (spi_done) begin
                        mic_q12 <= adc_q12;
                        state   <= S_STEP;
                    end
                end

                S_STEP: begin
                    sample_en <= 1'b1;          // kick off vocoder + advance NCO
                    state     <= S_VOC_WAIT;
                end

                S_VOC_WAIT: begin
                    if (vocoder_done) begin
                        dac_q12 <= vocoder_out;
                        state   <= S_DAC_REQ;
                    end
                end

                S_DAC_REQ: begin
                    spi_tx     <= dac_tx_word;
                    spi_start  <= 1'b1;
                    target_dac <= 1'b1;
                    state      <= S_DAC_WAIT;
                end

                S_DAC_WAIT: begin
                    if (spi_done)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // Mux the shared CS to the active slave (the other one is held high).
    assign adc_cs_n = target_dac ? 1'b1     : spi_cs_n;
    assign dac_cs_n = target_dac ? spi_cs_n : 1'b1;

    // ─── 1-bit Σ-Δ audio output (for Mike's Audio PMOD) ─────────────────────
    // First-order modulator on the biased unsigned value. uo_out[7] toggles
    // at chip clock rate; an external RC + amplifier integrates back to audio.
    reg [13:0] sd_acc;
    wire [12:0] vocoder_unsigned = vocoder_out ^ 13'h1000;

    always @(posedge clk) begin
        if (~rst_n)
            sd_acc <= 14'd0;
        else
            sd_acc <= {1'b0, sd_acc[12:0]} + {1'b0, vocoder_unsigned};
    end

    assign uo_out[7]   = sd_acc[13];
    assign uo_out[6:0] = {state, target_dac, spi_done, spi_cs_n, sck};

endmodule
