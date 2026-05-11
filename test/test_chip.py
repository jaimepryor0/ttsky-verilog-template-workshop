"""
Full chip-level integration test.

Drives the tt_um_JAIMEPRYOR0_VGA_YAY module exactly as it would see the world
on a TinyTapeout board:

  * The DUT thinks an **MCP3201 ADC** is on `uio[0..3]` (CS / MOSI / MISO / SCK).
    `MockMcp3201` queues a stream of 12-bit ADC codes and streams them back on
    MISO each time the chip drops ADC_CS.

  * The DUT thinks an **MCP4921 DAC** is on `uio[4] + uio[1] + uio[3]`.
    `MockMcp4921` captures every 16-bit word the chip writes and pulls the
    12-bit data field out.

  * The DUT's pitch byte is driven via `ui_in`.

For five different pitches the test runs the chip end-to-end on a 20 ms peak-
centred slice of hello.wav, recovers the captured 12-bit DAC stream, and
compares it sample-by-sample to a Python reference produced by
`vfp.process(chip_audio, chip_sawtooth)` -- both quantised the same way the
silicon would. We require **bit-exact** equality across all 5 pitch runs.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC  = os.path.abspath(os.path.join(_HERE, '..', 'src'))
sys.path.insert(0, _SRC)

import numpy as np  # noqa: E402

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge  # noqa: E402

import vocoder_fixed_point as vfp  # noqa: E402
from hw_int import hw_int  # noqa: E402


CLK_PERIOD_NS = 40                                  # 25 MHz chip clock
WINDOW_MS     = 20                                  # 20 ms ≈ 960 samples — keeps sim fast
AUDIO_PATH    = os.path.join(_SRC, 'hello.wav')
PITCHES       = [1, 2, 4, 8, 16]                    # ui_in values to sweep


# ─── Bit-exact helpers ─────────────────────────────────────────────────────

def chip_sawtooth(n_samples: int, pitch_byte: int) -> hw_int:
    """Replica of pitch.v's NCO. Pure integer math — matches the HW sample-for-sample.

    increment = pitch_byte << 24 (project.v ties ui_in into the top 8 bits of
    the 32-bit phase increment). Phase resets to 0x80000000 so sample 0 = -1.0.
    Output is the top 8 bits of the phase accumulator reinterpreted as Q1.7.
    """
    increment = (pitch_byte & 0xFF) << 24
    phase     = 0x80000000
    out       = np.zeros(n_samples, dtype=np.int64)
    for n in range(n_samples):
        top8    = (phase >> 24) & 0xFF
        out[n]  = top8 - 0x100 if (top8 & 0x80) else top8
        phase   = (phase + increment) & 0xFFFFFFFF
    return hw_int(out, bits=vfp.ADC_BITS, frac_bits=vfp.DATA_FRAC)


def q7_to_dac12(q7: np.ndarray) -> np.ndarray:
    """project.v: dac_data = {~dac_q7[7], dac_q7[6:0], 4'b0000}.

    Pads 4 zero LSBs onto the 8-bit Q1.7 value and flips the sign bit.
    """
    q7 = q7.astype(np.int64) & 0xFF                  # 8-bit two's complement
    return (((q7 ^ 0x80) << 4) & 0xFFF).astype(np.int64)


def adc12_to_q7(d12: np.ndarray) -> np.ndarray:
    """Chip's view of a 12-bit ADC code as Q1.7: flip MSB and drop bottom 4."""
    d12 = d12.astype(np.int64) & 0xFFF
    biased_8 = ((d12 ^ 0x800) >> 4) & 0xFF
    return np.where(biased_8 & 0x80, biased_8 - 0x100, biased_8)


def build_test_signal(window_ms: float = WINDOW_MS) -> hw_int:
    """Same peak-centred slice strategy as test_vocoder.py, shorter window."""
    full = vfp.load_audio(AUDIO_PATH)
    n    = int(round(window_ms * 1e-3 * vfp.FS))
    if len(full) <= n:
        return full
    peak  = int(np.argmax(np.abs(full.val)))
    start = max(0, min(len(full) - n, peak - n // 2))
    return full[start:start + n]


# ─── Mock SPI slaves ───────────────────────────────────────────────────────

class MockMcp3201:
    """SPI slave model of MCP3201.

    Wire format on MISO during the 16 SCK cycles after CS↓ is, per the
    datasheet: {sampling, null, D11..D0, trailing×2}. Master shifts those
    16 bits in MSB-first → rx_data, then takes bits [13:2] as the data.

    So all we have to do is pre-pack the 12-bit sample as
        {2'b00, D11..D0, 2'b00}
    and clock those 16 bits out MSB-first.
    """

    def __init__(self, dut):
        self.dut     = dut
        self.samples = []
        self.cursor  = 0
        self._task   = None
        self._restart()

    def _restart(self):
        if self._task is not None:
            self._task.cancel()
        self._task = cocotb.start_soon(self._run())

    def reset(self, samples):
        # Coerce to plain Python ints; cocotb refuses to deposit np.int64.
        self.samples = [int(s) for s in samples]
        self.cursor  = 0
        self.dut.miso_drive.value = 0
        self._restart()

    async def _run(self):
        dut = self.dut
        while True:
            dut.miso_drive.value = 0
            await FallingEdge(dut.adc_cs)

            if self.cursor < len(self.samples):
                sample = self.samples[self.cursor] & 0xFFF
                self.cursor += 1
            else:
                sample = 0

            shift_word = (sample & 0xFFF) << 2          # bits 13..2 carry data
            # First bit (MSB) must be valid before the first SCK rises.
            dut.miso_drive.value = (shift_word >> 15) & 1

            for k in range(16):
                await RisingEdge(dut.sck)               # master samples here
                if k < 15:
                    await FallingEdge(dut.sck)
                    dut.miso_drive.value = (shift_word >> (15 - (k + 1))) & 1

            await RisingEdge(dut.adc_cs)
            dut.miso_drive.value = 0


class MockMcp4921:
    """SPI slave model of MCP4921. Captures the 16-bit write word per
    transaction; we keep only the 12-bit data field (bottom 12 bits)."""

    def __init__(self, dut):
        self.dut       = dut
        self.captured  = []
        self._task     = None
        self._restart()

    def _restart(self):
        if self._task is not None:
            self._task.cancel()
        self._task = cocotb.start_soon(self._run())

    def reset(self):
        self.captured = []
        self._restart()

    async def _run(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.dac_cs)
            shift = 0
            for _ in range(16):
                await RisingEdge(dut.sck)
                shift = (shift << 1) | (int(dut.mosi.value) & 1)
            self.captured.append(shift & 0xFFF)


# ─── The test ──────────────────────────────────────────────────────────────

@cocotb.test()
async def chip_matches_python_over_pitches(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

    # Build stimulus
    audio_hw  = build_test_signal()
    n_samples = len(audio_hw)

    # The chip only sees 8 bits of the audio (12-bit ADC truncated to top 8).
    # Re-quantise the Python-side audio the same way so the reference matches
    # what the vocoder actually consumes inside the chip. The round-trip is
    # lossless for any 8-bit input, but we keep the dance explicit for clarity.
    adc_codes      = q7_to_dac12(audio_hw.val)
    chip_audio_q7  = adc12_to_q7(adc_codes)
    chip_audio_hw  = hw_int(chip_audio_q7, bits=vfp.ADC_BITS, frac_bits=vfp.DATA_FRAC)

    adc = MockMcp3201(dut)
    dac = MockMcp4921(dut)

    # One-time setup of static inputs
    dut.ena.value   = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 5)

    for pitch_byte in PITCHES:
        dut._log.info(f"─────────── pitch_byte = {pitch_byte} ───────────")

        # Reset chip + mocks together. We hold reset for several cycles to
        # ensure every register clears; then queue the ADC samples / clear the
        # DAC captures, set the new pitch, and release reset.
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 10)
        adc.reset(adc_codes)
        dac.reset()
        dut.ui_in.value = pitch_byte
        dut.rst_n.value = 1

        # Wait until the DAC has received all n_samples writes. The serial
        # multiplier puts the chip at ~1200 cycles/sample (256 ADC + 673 vocoder
        # + 256 DAC); 2000 leaves headroom.
        timeout_cycles = n_samples * 2_000 + 10_000
        for _ in range(timeout_cycles // 50):
            if len(dac.captured) >= n_samples:
                break
            await ClockCycles(dut.clk, 50)
        if len(dac.captured) < n_samples:
            raise TimeoutError(
                f"pitch={pitch_byte}: only {len(dac.captured)}/{n_samples} DAC "
                f"writes captured after {timeout_cycles} cycles"
            )

        # Python reference: same chip-quantised audio, sawtooth matched to the
        # exact NCO output, then quantise the vocoder output back through the
        # 12-bit DAC formula.
        saw_hw   = chip_sawtooth(n_samples, pitch_byte)
        ref      = vfp.process(chip_audio_hw, saw_hw)
        ref_dac  = q7_to_dac12(ref.val)

        captured = np.asarray(dac.captured[:n_samples], dtype=np.int64)
        diff     = captured - ref_dac
        bad      = np.flatnonzero(diff)
        if bad.size:
            head = bad[:8]
            details = "\n".join(
                f"  n={i:5d}: chip=0x{captured[i]:03X}  ref=0x{ref_dac[i]:03X}  "
                f"delta={int(diff[i]):+d}"
                for i in head
            )
            raise AssertionError(
                f"pitch={pitch_byte}: {bad.size}/{n_samples} DAC samples differ "
                f"from Python reference (first {len(head)} shown):\n{details}"
            )

        dut._log.info(
            f"pitch={pitch_byte}: all {n_samples} DAC samples match Python reference."
        )

    dut._log.info(
        f"All {len(PITCHES)} pitches produced bit-exact DAC output against the Python model."
    )


# ─── Runner ────────────────────────────────────────────────────────────────

def main():
    from cocotb_tools.runner import get_runner

    sim    = os.environ.get("SIM", "verilator")
    runner = get_runner(sim)
    runner.build(
        sources=[
            os.path.join(_SRC,  'filter.v'),
            os.path.join(_SRC,  'pitch.v'),
            os.path.join(_SRC,  'vocoder.v'),
            os.path.join(_SRC,  'spi_master.v'),
            os.path.join(_SRC,  'project.v'),
            os.path.join(_HERE, 'tb_project.v'),
        ],
        hdl_toplevel='tb_project',
        build_dir=os.path.join(_HERE, 'sim_build', 'chip'),
        timescale=('1ns', '1ps'),
        # Verilator's -G overrides come in as 32-bit ints regardless of the
        # toplevel parameter widths; we don't override params here but keep the
        # WIDTHTRUNC suppression for parity with the other test runners.
        build_args=['-Wno-WIDTHTRUNC'],
        always=True,
    )
    runner.test(
        hdl_toplevel='tb_project',
        test_module='test_chip',
        test_dir=_HERE,
    )


if __name__ == '__main__':
    main()
