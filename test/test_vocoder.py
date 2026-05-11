"""
Cocotb test: drive the vocoder RTL one sample at a time and assert every
output sample matches vocoder_fixed_point.py exactly.

This module is both:
  - the @cocotb.test() target loaded inside the simulator
  - the runner script that builds + launches the simulation
    (run `python test_vocoder.py` from this directory, or via pytest)

The vocoder uses a shared 16x16 multiplier sequenced over 9 internal
biquads, so each sample takes ~49 chip clocks. Drive `start` for one
cycle with the desired mic/saw inputs, then wait for `done` to sample
`out`.

Coefficient parameters are computed in Python at build time and
passed straight to the toplevel via cocotb_tools.runner -- no
generated `include files.
"""
import os
import sys

# Make src/ importable so cocotb code and the runner share the same
# behavioural model.
_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC  = os.path.abspath(os.path.join(_HERE, '..', 'src'))
sys.path.insert(0, _SRC)

import numpy as np  # noqa: E402

import vocoder_fixed_point as vfp  # noqa: E402
from hw_int import hw_int  # noqa: E402


# -- Coefficient design (shared by runner and Python reference) -------------

def _design_parameters() -> dict[str, int]:
    """Compute all 17 Q2.14 coefficient ints to pass as toplevel parameters."""
    params: dict[str, int] = {}
    for band_idx, (flo, fhi) in enumerate(vfp.VOICE_BANDS, start=1):
        sections = vfp._design_bandpass(flo, fhi)
        if len(sections) != 1:
            raise RuntimeError(
                f"Band {band_idx} produced {len(sections)} SOS sections; the RTL "
                "models a single biquad per band. Set FILTER_ORDER = 1."
            )
        b_hw, a_hw = sections[0]
        params[f"B{band_idx}_b0"] = int(b_hw.val[0])
        params[f"B{band_idx}_b1"] = int(b_hw.val[1])
        params[f"B{band_idx}_b2"] = int(b_hw.val[2])
        params[f"B{band_idx}_a1"] = int(a_hw.val[1])
        params[f"B{band_idx}_a2"] = int(a_hw.val[2])

    scale = 2 ** vfp.COEF_FRAC
    params["ENV_b0"] = int(round((1.0 - vfp.ENV_ALPHA) * scale))
    params["ENV_a1"] = int(round(-vfp.ENV_ALPHA * scale))
    return params


# -- Cocotb test (runs inside the simulator) --------------------------------

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer  # noqa: E402


CLOCK_PERIOD_NS = 100
WINDOW_MS       = 100   # length of audio slice we run through the DUT
AUDIO_PATH      = os.path.join(_SRC, 'hello.wav')


def _build_test_signal(audio_path: str = AUDIO_PATH,
                       window_ms: float = WINDOW_MS) -> hw_int:
    """Take a window_ms slice of audio_path centred on its absolute peak."""
    full = vfp.load_audio(audio_path)
    n = int(round(window_ms * 1e-3 * vfp.FS))
    if len(full) <= n:
        return full
    peak  = int(np.argmax(np.abs(full.val)))
    start = max(0, min(len(full) - n, peak - n // 2))
    return full[start:start + n]


async def _reset(dut):
    dut.rst_n.value = 0
    dut.mic.value   = 0
    dut.saw.value   = 0
    dut.start.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await FallingEdge(dut.clk)


async def _process_one_sample(dut, mic_val: int, saw_val: int) -> int:
    """Drive one sample through the start/done handshake and return out."""
    dut.mic.value   = int(mic_val)
    dut.saw.value   = int(saw_val)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    await RisingEdge(dut.done)
    # `done` and `out` commit on the same edge; out is already valid here.
    return dut.out.value.to_signed()


@cocotb.test()
async def vocoder_matches_python(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, unit="ns").start())
    await _reset(dut)

    audio_hw  = _build_test_signal()
    n_samples = len(audio_hw)
    saw_hw    = vfp._generate_sawtooth(n_samples)
    expected  = vfp.process(audio_hw, saw_hw)

    audio_int = audio_hw.val.astype(np.int64)
    saw_int   = saw_hw.val.astype(np.int64)
    exp_int   = expected.val.astype(np.int64)

    y_dut = np.zeros(n_samples, dtype=np.int64)
    for n in range(n_samples):
        y_dut[n] = await _process_one_sample(dut,
                                             int(audio_int[n]),
                                             int(saw_int[n]))

    diff = y_dut - exp_int
    bad  = np.flatnonzero(diff)
    if bad.size:
        head = bad[:8]
        details = "\n".join(
            f"  n={i:5d}: dut={y_dut[i]:7d}  ref={exp_int[i]:7d}  delta={diff[i]:+d}"
            for i in head
        )
        raise AssertionError(
            f"DUT differs from Python reference at {bad.size}/{n_samples} "
            f"samples (first {len(head)} shown):\n{details}"
        )

    dut._log.info(f"All {n_samples} samples match Python reference exactly.")


@cocotb.test()
async def vocoder_matches_python_with_idle_gaps(dut):
    """Same comparison but with random idle cycles between start pulses,
    to verify the FSM correctly idles waiting for the next sample.
    """
    import random
    random.seed(0xCAFE)

    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, unit="ns").start())
    await _reset(dut)

    audio_hw  = _build_test_signal()
    n_samples = len(audio_hw)
    saw_hw    = vfp._generate_sawtooth(n_samples)
    expected  = vfp.process(audio_hw, saw_hw)

    audio_int = audio_hw.val.astype(np.int64)
    saw_int   = saw_hw.val.astype(np.int64)
    exp_int   = expected.val.astype(np.int64)

    y_dut = np.zeros(n_samples, dtype=np.int64)
    for n in range(n_samples):
        # Idle for a random number of cycles with start held low
        gap = random.randint(0, 4)
        if gap:
            await ClockCycles(dut.clk, gap)
        y_dut[n] = await _process_one_sample(dut,
                                             int(audio_int[n]),
                                             int(saw_int[n]))

    diff = y_dut - exp_int
    bad  = np.flatnonzero(diff)
    if bad.size:
        head = bad[:8]
        details = "\n".join(
            f"  n={i:5d}: dut={y_dut[i]:7d}  ref={exp_int[i]:7d}  delta={diff[i]:+d}"
            for i in head
        )
        raise AssertionError(
            f"DUT (idle-gap) differs from Python reference at {bad.size}/{n_samples} "
            f"samples (first {len(head)} shown):\n{details}"
        )

    dut._log.info(f"All {n_samples} samples match with idle gaps between starts.")


# -- Runner entry point (runs outside the simulator) ------------------------

def main() -> None:
    from cocotb_tools.runner import get_runner

    sim = os.environ.get("SIM", "verilator")
    runner = get_runner(sim)

    runner.build(
        sources=[
            os.path.join(_SRC,  'vocoder.v'),
            os.path.join(_HERE, 'tb_vocoder.v'),
        ],
        hdl_toplevel='tb_vocoder',
        parameters=_design_parameters(),
        build_dir=os.path.join(_HERE, 'sim_build', 'vocoder'),
        timescale=('1ns', '1ps'),
        # Verilator narrows 32-bit -G overrides to the 16-bit signed
        # toplevel parameters and warns; that narrowing is exactly what
        # we want.
        build_args=['-Wno-WIDTHTRUNC'],
        always=True,
    )
    runner.test(
        hdl_toplevel='tb_vocoder',
        test_module='test_vocoder',
        test_dir=_HERE,
    )


if __name__ == '__main__':
    main()
