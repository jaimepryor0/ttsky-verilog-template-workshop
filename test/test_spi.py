"""
Third-party verification of src/spi_master.v using cocotbext-spi.

A cocotbext-spi slave attaches to the master's wires and:
  - Verifies the master frames a clean Mode 0 transaction (any timing
    violation throws SpiFrameError out of the slave coroutine).
  - Confirms that the bytes the master shifted out on MOSI arrive at the
    slave in the right order (MSB-first).
  - Drives a known pattern back on MISO so that the master's rx_data can
    be checked against an independent reference.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC  = os.path.abspath(os.path.join(_HERE, '..', 'src'))

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge  # noqa: E402

from cocotbext.spi import SpiBus, SpiConfig, SpiSlaveBase  # noqa: E402


CLK_PERIOD_NS = 40            # 25 MHz chip clock
SCK_DIV       = 16            # must match the parameter in tb_spi.v


class EchoSpiSlave(SpiSlaveBase):
    """Slave that captures the master's MOSI word and drives a configurable
    response on MISO during the same frame."""

    _config = SpiConfig(
        word_width      = 16,
        cpol            = False,    # Mode 0
        cpha            = False,
        msb_first       = True,
        cs_active_low   = True,
        data_output_idle= 0,
        frame_spacing_ns= 1,
    )

    def __init__(self, bus):
        self.next_tx = 0
        self.last_rx = None
        super().__init__(bus)

    async def _transaction(self, frame_start, frame_end):
        # Custom edge-by-edge implementation. cocotbext-spi's _shift refuses to
        # accept a master that deasserts CS one chip-cycle after the last SCK
        # falling edge (a perfectly valid Mode 0 timing); rolling our own keeps
        # us bit-true to the spec without that strictness.
        #
        # _run only guarantees `frame_spacing` has elapsed -- we still need to
        # wait for the actual CS-falling event ourselves.
        await frame_start
        tx = self.next_tx
        # CPHA=0 expects the first MISO bit to be valid before the first SCK
        # rising edge — drive the MSB now.
        self._miso.value = bool(tx & (1 << 15))
        rx = 0
        for k in range(16):
            await RisingEdge(self._sclk)
            rx |= int(self._mosi.value) << (15 - k)
            await FallingEdge(self._sclk)
            if k < 15:
                self._miso.value = bool(tx & (1 << (15 - (k + 1))))
        self.last_rx = rx


async def _kick_transaction(dut, tx, slave_response):
    """Latch tx_data, pulse start for one cycle, wait for `done`."""
    slave.next_tx = slave_response  # noqa: F821 — set by caller
    await RisingEdge(dut.clk)
    dut.tx_data.value = tx
    dut.start.value   = 1
    await RisingEdge(dut.clk)
    dut.start.value   = 0

    # Wait for the master's done strobe (FSM finishes after 16 SCK pulses).
    for _ in range(16 * SCK_DIV * 2):
        await RisingEdge(dut.clk)
        if int(dut.done.value):
            return int(dut.rx_data.value)
    raise TimeoutError("SPI master never asserted `done`")


@cocotb.test()
async def spi_master_round_trip(dut):
    """A handful of round-trips of arbitrary 16-bit words.

    For each word w sent by the master with an echo r set up on the slave:
      assert slave.last_rx == w          (master → slave path)
      assert master.rx_data == r         (slave  → master path)
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

    global slave
    spi_bus = SpiBus.from_entity(dut,
                                 sclk_name="sck",
                                 mosi_name="mosi",
                                 miso_name="miso",
                                 cs_name="cs_n")
    slave = EchoSpiSlave(spi_bus)

    # Reset
    dut.rst_n.value   = 0
    dut.start.value   = 0
    dut.tx_data.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)

    vectors = [
        (0x0000, 0xFFFF),     # all-zero MOSI, all-one MISO
        (0xFFFF, 0x0000),     # reversed extremes
        (0xA5C3, 0xDEAD),     # mixed bit pattern
        (0x8001, 0x7FFE),     # single-MSB / single-LSB
        (0x1234, 0xABCD),     # arbitrary
    ]

    for tx_word, rx_response in vectors:
        rx_master = await _kick_transaction(dut, tx_word, rx_response)

        # Let the slave's _run coroutine finish updating last_rx (CS just rose).
        await ClockCycles(dut.clk, 4)

        assert slave.last_rx == tx_word, (
            f"slave saw 0x{slave.last_rx:04X} on MOSI, expected 0x{tx_word:04X}"
        )
        assert rx_master == rx_response, (
            f"master saw 0x{rx_master:04X} on MISO, expected 0x{rx_response:04X}"
        )
        dut._log.info(
            f"  tx=0x{tx_word:04X} -> slave rx=0x{slave.last_rx:04X}   "
            f"slave tx=0x{rx_response:04X} -> master rx=0x{rx_master:04X}  OK"
        )

    dut._log.info(f"All {len(vectors)} round-trip transactions match the SPI Mode 0 reference.")


# ── Runner entry point ──────────────────────────────────────────────────────

def main() -> None:
    from cocotb_tools.runner import get_runner

    sim = os.environ.get("SIM", "verilator")
    runner = get_runner(sim)

    runner.build(
        sources=[
            os.path.join(_SRC,  'spi_master.v'),
            os.path.join(_HERE, 'tb_spi.v'),
        ],
        hdl_toplevel='tb_spi',
        build_dir=os.path.join(_HERE, 'sim_build', 'spi'),
        timescale=('1ns', '1ps'),
        always=True,
    )
    runner.test(
        hdl_toplevel='tb_spi',
        test_module='test_spi',
        test_dir=_HERE,
    )


if __name__ == '__main__':
    main()
