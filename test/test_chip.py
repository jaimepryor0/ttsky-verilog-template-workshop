"""
Full chip-level integration test for the parallel-audio top.

Protocol per sample:
  1. Drive ui_in[6:0] with the 7-bit signed Q1.6 sample value (valid bit 0).
  2. Raise ui_in[7] to give a rising edge -- this latches mic and starts
     a vocoder cycle.
  3. Wait for the rising edge of uo_out[7]; uo_out[6:0] now holds the
     7-bit Q1.6 result of that sample.
  4. Lower ui_in[7] before driving the next sample so the next 0->1
     transition is seen as a fresh edge.

The Python reference uses vfp.process() on the *padded* 8-bit Q1.7 signal
the chip actually sees (LSB = 0), then drops the LSB of the output to match
the 7-bit pin format. Bit-exact equality is required across all pitches.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC  = os.path.abspath(os.path.join(_HERE, '..', 'src'))
sys.path.insert(0, _SRC)

import numpy as np  # noqa: E402

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, RisingEdge  # noqa: E402

import vocoder_fixed_point as vfp  # noqa: E402
from hw_int import hw_int  # noqa: E402


CLK_PERIOD_NS = 40                                  # 25 MHz chip clock
WINDOW_MS     = 20                                  # 20 ms ≈ 960 samples
AUDIO_PATH    = os.path.join(_SRC, 'hello.wav')
# uio_byte = {mode[1:0], pitch[5:0]}. Spread the test across all four
# carrier modes (0=saw, 1=saw-down, 2=square, 3=triangle) and a few
# pitches to exercise both axes of the new pin layout.
UIO_BYTES     = [0x01, 0x02, 0x04, 0x08, 0x10,      # mode 0 (saw up)
                 0x42, 0x84, 0xC4]                  # modes 1..3 at pitch=2,4,4


# ─── Bit-exact helpers ─────────────────────────────────────────────────────

def chip_carrier(n_samples: int, uio_byte: int) -> hw_int:
    """Replica of pitch.v: 8-bit NCO + waveform selector.

    uio_byte layout matches the chip:
      bits [7:6] = waveform mode  (0=saw up, 1=saw down, 2=square, 3=triangle)
      bits [5:0] = 6-bit pitch byte; project.v promotes it to the top 6 bits
                   of an 8-bit phase increment (multiply by 4).
    """
    mode   = (uio_byte >> 6) & 0x3
    pitch6 = uio_byte & 0x3F
    inc    = (pitch6 << 2) & 0xFF
    phase  = 0x80
    out    = np.zeros(n_samples, dtype=np.int64)
    for n in range(n_samples):
        saw_u  = phase & 0xFF
        saw_s  = saw_u - 0x100 if (saw_u & 0x80) else saw_u
        abs_saw = -saw_s if saw_s < 0 else saw_s        # 0..128
        if mode == 0:
            out[n] = saw_s
        elif mode == 1:
            out[n] = (~saw_s) & 0xFF                     # bit-invert
            if out[n] & 0x80:
                out[n] -= 0x100
        elif mode == 2:
            out[n] = -64 if (saw_u & 0x80) else 63
        else:  # mode == 3 (triangle)
            out[n] = abs_saw - 64
        phase = (phase + inc) & 0xFF
    return hw_int(out, bits=vfp.ADC_BITS, frac_bits=vfp.DATA_FRAC)


def q7_to_pin7(q7_8: np.ndarray) -> np.ndarray:
    """8-bit Q1.7 -> 7-bit Q1.6 by arithmetic right shift (drops the LSB)."""
    return (q7_8.astype(np.int64) >> 1).astype(np.int64)


def pin7_to_chip_q7(pin7: np.ndarray) -> np.ndarray:
    """The chip pads with a zero LSB:  mic = {ui_in[6:0], 1'b0} (signed)."""
    return (pin7.astype(np.int64) << 1).astype(np.int64)


def signed_7bit(u7: int) -> int:
    """Read a 7-bit two's-complement value out of a 7-bit unsigned field."""
    return u7 - 0x80 if (u7 & 0x40) else u7


def build_test_signal(window_ms: float = WINDOW_MS) -> hw_int:
    full = vfp.load_audio(AUDIO_PATH)
    n    = int(round(window_ms * 1e-3 * vfp.FS))
    if len(full) <= n:
        return full
    peak  = int(np.argmax(np.abs(full.val)))
    start = max(0, min(len(full) - n, peak - n // 2))
    return full[start:start + n]


# ─── The test ──────────────────────────────────────────────────────────────

@cocotb.test()
async def chip_matches_python_over_pitches(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

    audio_hw  = build_test_signal()
    n_samples = len(audio_hw)

    # The producer only has 7 bits of resolution, so the LSB of the source
    # audio is lost at the pin. We replicate that on the Python side by
    # right-shifting once, then left-shifting back to get the exact 8-bit
    # Q1.7 sequence the vocoder will see internally.
    audio_pin7    = q7_to_pin7(audio_hw.val)                # signed 7-bit
    chip_mic_q7   = pin7_to_chip_q7(audio_pin7)             # signed 8-bit Q1.7
    chip_audio_hw = hw_int(chip_mic_q7, bits=vfp.ADC_BITS, frac_bits=vfp.DATA_FRAC)

    dut.ena.value    = 1
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)

    for uio_byte in UIO_BYTES:
        mode    = (uio_byte >> 6) & 0x3
        pitch6  =  uio_byte       & 0x3F
        mode_lbl = ['saw', 'saw-down', 'square', 'triangle'][mode]
        dut._log.info(f"─────────── uio=0x{uio_byte:02X} (mode={mode_lbl}, pitch={pitch6}) ───────────")

        dut.rst_n.value  = 0
        await ClockCycles(dut.clk, 10)
        dut.uio_in.value = uio_byte
        dut.ui_in.value  = 0
        dut.rst_n.value  = 1
        await ClockCycles(dut.clk, 2)

        captured = np.zeros(n_samples, dtype=np.int64)
        for n in range(n_samples):
            # 7-bit two's complement of the sample, sitting in ui_in[6:0].
            pin_val = int(audio_pin7[n]) & 0x7F

            # Drop the valid bit (and update the sample bits underneath it)
            # so the next 0->1 transition is a clean rising edge.
            dut.ui_in.value = pin_val
            await ClockCycles(dut.clk, 1)
            # Raise valid -> rising edge starts the vocoder.
            dut.ui_in.value = pin_val | 0x80

            # Wait for the chip to emit a fresh output sample.
            await RisingEdge(dut.out_valid)
            captured[n] = signed_7bit(int(dut.uo_out.value) & 0x7F)

        # Python reference: feed the same padded 8-bit audio + matching
        # sawtooth into vfp.process, then drop the LSB to land back in the
        # 7-bit pin format.
        saw_hw   = chip_carrier(n_samples, uio_byte)
        ref_q7   = vfp.process(chip_audio_hw, saw_hw)
        ref_pin7 = q7_to_pin7(ref_q7.val)

        diff = captured - ref_pin7
        bad  = np.flatnonzero(diff)
        if bad.size:
            head = bad[:8]
            details = "\n".join(
                f"  n={i:5d}: chip={captured[i]:4d}  ref={ref_pin7[i]:4d}  "
                f"delta={int(diff[i]):+d}"
                for i in head
            )
            raise AssertionError(
                f"uio=0x{uio_byte:02X}: {bad.size}/{n_samples} samples differ "
                f"from Python reference (first {len(head)} shown):\n{details}"
            )

        dut._log.info(
            f"uio=0x{uio_byte:02X}: all {n_samples} samples match Python reference."
        )

    dut._log.info(
        f"All {len(UIO_BYTES)} uio configurations produced bit-exact output against the Python model."
    )


# ─── Runner ────────────────────────────────────────────────────────────────

def main():
    from cocotb_tools.runner import get_runner

    sim    = os.environ.get("SIM", "verilator")
    runner = get_runner(sim)
    runner.build(
        sources=[
            os.path.join(_SRC,  'pitch.v'),
            os.path.join(_SRC,  'vocoder.v'),
            os.path.join(_SRC,  'project.v'),
            os.path.join(_HERE, 'tb_project.v'),
        ],
        hdl_toplevel='tb_project',
        build_dir=os.path.join(_HERE, 'sim_build', 'chip'),
        timescale=('1ns', '1ps'),
        # Verilator narrows wider -G overrides to the top's parameter widths
        # and warns; suppress for parity with the other test runners.
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
