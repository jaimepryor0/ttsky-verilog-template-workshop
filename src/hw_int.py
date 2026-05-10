import numpy as np 

class hw_int: 
    def __init__(self, val, bits=8, numrange: tuple = (0, 25)):
        self.bits = bits
        self.bitmaxrange = 2**self.bits-1 # self.bits creates range from 0-self.bit_max_range
        self.minrange = numrange[0]
        self.maxrange = numrange[1]
        self.gridsize = (self.maxrange-self.minrange)/self.bitmaxrange
        self.val = self.cast(val)
        self.overflow = np.mod(val, self.gridsize)
        self.dac = self.uncast(self.val)

    def cast(self, x): 
        # cast a number to 8 bit representation 
        # for the input cast to the closest value on the grid (round to closest)
        cast = int(np.round(x/self.gridsize))
        if cast > self.bitmaxrange: 
            cast = int(self.bitmaxrange)
        return cast
    def uncast(self, x):
        return x*self.gridsize
    
    def __mul__ (self, other : 'hw_int'):
        try: 
            return hw_int(self.val * other.val, bits=self.bits, numrange=(self.minrange, self.maxrange))
        except AttributeError: 
            return hw_int(val=self.val*other, bits=self.bits, numrange=(self.minrange, self.maxrange))