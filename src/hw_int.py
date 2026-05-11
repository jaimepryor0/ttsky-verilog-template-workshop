from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike, NDArray

class hw_int:
    """
    Signed fixed-point integer model for hardware arithmetic.
    val may be a scalar or a numpy array — all operations are element-wise.

    Representation: bits-wide two's complement, frac_bits binary fractional bits.
    Real value = val / 2**frac_bits.
    """

    bits: int
    frac_bits: int
    modulus: int
    val: NDArray[np.int64]

    @staticmethod
    def from_float(val: ArrayLike, bits: int, frac_bits: int) -> hw_int:
        scaled = np.round(np.asarray(val, dtype=float) * 2**frac_bits).astype(np.int64)
        return hw_int(scaled, bits, frac_bits)

    def __init__(self, val: ArrayLike, bits: int = 8, frac_bits: int = 0) -> None:
        if (bits > 64 or bits <= 0):
            raise ValueError(f"{bits=}, but only 1 to 64 bits supported")
        if (frac_bits > bits or frac_bits < 0):
            raise ValueError(f"{frac_bits=}, but only 0 to {bits=} bits supported")
        self.bits = bits
        self.frac_bits = frac_bits
        self.modulus = 2**bits
        self.val = self._cast(val)

    def _cast(self, x: ArrayLike) -> NDArray[np.int64]:
        x = np.asarray(x, dtype=np.int64) % self.modulus
        # Two's complement wrap into [-2^(bits-1), 2^(bits-1) - 1]
        return np.where(x >= self.modulus // 2, x - self.modulus, x)

    def to_float(self) -> NDArray[np.float64]:
        return self.val / 2**self.frac_bits

    def truncate(self, bits: int, frac_bits: int) -> hw_int:
        """Requantize to fewer bits — models a hardware truncation/rounding stage."""
        shift = self.frac_bits - frac_bits
        truncated = (self.val >> shift) if shift >= 0 else (self.val << -shift)
        return hw_int(truncated, bits=bits, frac_bits=frac_bits)

    def __mul__(self, other: hw_int) -> hw_int:
        if isinstance(other, hw_int):
            # Full-precision product: bits and frac_bits both grow
            return hw_int(self.val * other.val,
                          bits=self.bits + other.bits,
                          frac_bits=self.frac_bits + other.frac_bits)
        raise TypeError(f"Cannot multiply hw_int with {type(other)}")

    def __add__(self, other: hw_int) -> hw_int:
        if isinstance(other, hw_int):
            if self.frac_bits != other.frac_bits:
                raise ValueError(f"frac_bits mismatch: {self.frac_bits} vs {other.frac_bits}")
            return hw_int(self.val + other.val,
                          bits=max(self.bits, other.bits) + 1,
                          frac_bits=self.frac_bits)
        raise TypeError(f"Cannot add hw_int with {type(other)}")

    def __neg__(self) -> hw_int:
        return hw_int(-self.val, bits=self.bits, frac_bits=self.frac_bits)

    def __abs__(self) -> hw_int:
        # Most-negative value has no positive counterpart at same width — wraps like hardware.
        return hw_int(np.abs(self.val), bits=self.bits, frac_bits=self.frac_bits)

    def __sub__(self, other: hw_int) -> hw_int:
        if isinstance(other, hw_int):
            if self.frac_bits != other.frac_bits:
                raise ValueError(f"frac_bits mismatch: {self.frac_bits} vs {other.frac_bits}")
            return hw_int(self.val - other.val,
                          bits=max(self.bits, other.bits) + 1,
                          frac_bits=self.frac_bits)
        raise TypeError(f"Cannot subtract hw_int with {type(other)}")

    def __getitem__(self, idx: int | slice) -> hw_int:
        return hw_int(self.val[idx], bits=self.bits, frac_bits=self.frac_bits)

    def __len__(self) -> int:
        return len(self.val)

    def __repr__(self) -> str:
        int_bits = self.bits - self.frac_bits - 1  # excludes sign bit
        return f"{self.to_float()}(Q{int_bits}.{self.frac_bits}"
