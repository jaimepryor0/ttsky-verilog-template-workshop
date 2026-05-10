import scipy.signal as sig
import numpy as np 

def vocoder_behavioural(x, a1, a2, a3, b1, b2, b3, saw_periods):
    # Calculate filtering action using lfilter function 
    mic_iir_1 = sig.lfilter(b1, a1, x=x)
    mic_iir_2 = sig.lfilter(b2, a2, x=x)
    mic_iir_3 = sig.lfilter(b3, a3, x=x)


    # Generate sawtooth 
    samples = len(x)
    n = np.linspace(0, saw_periods*2*np.pi, samples)
    pitch = sig.sawtooth(n, 0.2)

    # Calculate filtering action using lfilter function 
    pitch_iir_1 = sig.lfilter(b1, a1, x=pitch)
    pitch_iir_2 = sig.lfilter(b2, a2, x=pitch)
    pitch_iir_3 = sig.lfilter(b3, a3, x=pitch)

    return [(mic_iir_1[i]*pitch_iir_1[i] + mic_iir_2[i]*pitch_iir_2[i] + mic_iir_3[i]*pitch_iir_3[i]) for i in range(len(mic_iir_1))]
