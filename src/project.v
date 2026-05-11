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
    reg                sample_en;           // 1-cycle start pulse per sample
    wire signed [7:0]  saw_q7;
    wire signed [7:0]  vocoder_out;
    wire               vocoder_done;        // pulses when vocoder_out is fresh

    pitch u_pitch (
        .clk      (clk),
        .rst_n    (rst_n),
        .en       (sample_en),
        .increment(ui_in),
        .out      (saw_q7)
    );

    // Coefficients (Q2.6) generated from vocoder_fixed_point.VOICE_BANDS at FS=48k.
    // ENV_b0=1 (= 1/64) because (1-alpha) at the chosen 100 Hz cutoff lands
    // just inside Q2.6 resolution -- 20 Hz would quantise to 0.
    vocoder #(
        .B1_b0( 8'sd3  ), .B1_b2(-8'sd3 ),
        .B1_a1(-8'sd121), .B1_a2( 8'sd58),
        .B2_b0( 8'sd6  ), .B2_b2(-8'sd6 ),
        .B2_a1(-8'sd116), .B2_a2( 8'sd53),
        .B3_b0( 8'sd14 ), .B3_b2(-8'sd14),
        .B3_a1(-8'sd91 ), .B3_a2( 8'sd37),
        .ENV_b0(8'sd1),   .ENV_a1(-8'sd63)
    ) u_vocoder (
        .clk  (clk),
        .rst_n(rst_n),
        .start(sample_en),
        .done (vocoder_done),
        .mic  (adc_q7),
        .saw  (saw_q7),
        .out  (vocoder_out)
    );

    // ─── Sample format conversions ──────────────────────────────────────────
    // MCP3201: 16 SCK with the 12 data bits in spi_rx[13:2] (null bit at [14],
    // sampling/trailing bits elsewhere). Bias is around 2048 = 0V. Drop the
    // bottom 4 ADC bits and toggle the MSB to produce 8-bit signed Q1.7.
    /* verilator lint_off UNUSEDSIGNAL */
    wire [11:0] adc_unsigned = spi_rx[13:2];   // bottom 4 bits dropped on truncate
    /* verilator lint_on UNUSEDSIGNAL */
    wire signed [7:0] adc_q7 = {~adc_unsigned[11], adc_unsigned[10:4]};

    // MCP4921 write word: {A/B, BUF, ~GA, ~SHDN, D11..D0}. Channel A,
    // unbuffered, 1× gain, active = 4'b0011. Convert 8-bit Q1.7 to 12-bit
    // unsigned offset binary by toggling the sign bit and zero-padding the
    // bottom 4 LSBs. `vocoder_out` is a registered output of the vocoder
    // module and stays valid from `done` until the next sample.
    wire [11:0] dac_data    = {~vocoder_out[7], vocoder_out[6:0], 4'b0000};
    wire [15:0] dac_tx_word = {4'b0011, dac_data};

    // ─── Controller FSM ─────────────────────────────────────────────────────
    //   IDLE → ADC_WAIT → STEP → VOC_WAIT → DAC_REQ → DAC_WAIT → IDLE
    // STEP pulses sample_en for one cycle, kicking off the serial vocoder
    // (and advancing the pitch NCO). VOC_WAIT blocks until vocoder_done
    // pulses; vocoder_out is then valid until the next sample, so the DAC
    // SPI write can read it directly without a holding register.
    localparam S_IDLE     = 3'd0;
    localparam S_ADC_WAIT = 3'd1;
    localparam S_STEP     = 3'd2;
    localparam S_VOC_WAIT = 3'd3;
    localparam S_DAC_REQ  = 3'd4;
    localparam S_DAC_WAIT = 3'd5;

    reg [2:0] state;

    always @(posedge clk) begin
        if (~rst_n) begin
            state      <= S_IDLE;
            spi_start  <= 1'b0;
            spi_tx     <= 16'h0000;
            sample_en  <= 1'b0;
        end else begin
            spi_start <= 1'b0;             // default — pulse only when needed
            sample_en <= 1'b0;

            case (state)
                S_IDLE: begin
                    spi_tx     <= 16'h0000;
                    spi_start  <= 1'b1;    // kick off ADC read
                    state      <= S_ADC_WAIT;
                end

                S_ADC_WAIT: begin
                    if (spi_done)
                        state <= S_STEP;
                end

                S_STEP: begin
                    sample_en <= 1'b1;          // kick off vocoder + advance NCO
                    state     <= S_VOC_WAIT;
                end

                S_VOC_WAIT: begin
                    if (vocoder_done)
                        state <= S_DAC_REQ;
                end

                S_DAC_REQ: begin
                    spi_tx     <= dac_tx_word;
                    spi_start  <= 1'b1;
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
    // target_dac is high during DAC SPI states only; derived from `state`
    // instead of a dedicated register.
    wire target_dac = (state == S_DAC_REQ) || (state == S_DAC_WAIT);
    assign adc_cs_n = target_dac ? 1'b1     : spi_cs_n;
    assign dac_cs_n = target_dac ? spi_cs_n : 1'b1;

    // ─── 1-bit Σ-Δ audio output (for Mike's Audio PMOD) ─────────────────────
    // First-order modulator on the biased unsigned value. uo_out[7] toggles
    // at chip clock rate; an external RC + amplifier integrates back to audio.
    reg [8:0] sd_acc;
    wire [7:0] vocoder_unsigned = vocoder_out ^ 8'h80;

    always @(posedge clk) begin
        if (~rst_n)
            sd_acc <= 9'd0;
        else
            sd_acc <= {1'b0, sd_acc[7:0]} + {1'b0, vocoder_unsigned};
    end

    assign uo_out[7]   = sd_acc[8];
    assign uo_out[6:0] = {state, target_dac, spi_done, spi_cs_n, sck};

endmodule
