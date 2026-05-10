import numpy as np

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