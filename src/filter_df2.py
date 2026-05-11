import numpy as np
from hw_int import hw_int


def filter_df2_hw(b: hw_int, a: hw_int, x: hw_int) -> hw_int:
    """
    Direct Form II IIR filter over fixed-point hw_int arrays.

    Assumes a[0] = 1 (normalised denominator). Each multiply is
    truncated back to the input data format before accumulation,
    modelling the rounding stage after the hardware multiplier.
    """
    assert len(b) == len(a), "b and a must have equal length"

    data_bits, data_frac = x.bits, x.frac_bits
    n_states = len(a) - 1
    zero = hw_int(0, bits=data_bits, frac_bits=data_frac)
    s: list[hw_int] = [zero] * n_states

    y_vals = np.empty(len(x), dtype=np.int64)

    for n in range(len(x)):
        xn = x[n]

        # y[n] = x[n]*b[0] + s[0]
        yn = (xn * b[0]).truncate(data_bits, data_frac)
        if n_states > 0:
            yn = (yn + s[0]).truncate(data_bits, data_frac)
        y_vals[n] = int(yn.val)

        # Update state variables s[0] … s[-1]
        for i in range(n_states - 1):
            t1 = (xn * b[i + 1]).truncate(data_bits, data_frac)
            t2 = (yn * a[i + 1]).truncate(data_bits, data_frac)
            s[i] = (t1 - t2 + s[i + 1]).truncate(data_bits, data_frac)

        if n_states > 0:
            t1 = (xn * b[-1]).truncate(data_bits, data_frac)
            t2 = (yn * a[-1]).truncate(data_bits, data_frac)
            s[-1] = (t1 - t2).truncate(data_bits, data_frac)

    return hw_int(y_vals, bits=data_bits, frac_bits=data_frac)


def filter_df2(b, a, x):

    x = np.asarray(x, dtype=np.float64)
    y = np.zeros(len(x), dtype=np.float64)

    # Set initial values for state variables (stored in delay registers)
    s = np.zeros(len(a)-1)

    # Start by assuming second order 

    for n in range(len(x)):

        # Calculate output
        y[n] = (x[n]*b[0] + s[0])/a[0]

        # Update intermediate state variables 
        for i in range(len(s)-1):
            s[i] = x[n]*b[i+1] - y[n]*a[i+1] + s[i+1]
        
        # Update last state variable
        s[-1] = x[n]*b[-1] - y[n]*a[-1] 
    
    return y