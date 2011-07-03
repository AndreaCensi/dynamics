import numpy as np

def list_of_dynamics():
    from dynamics import *
    yield d_rb_R1_v 
    yield d_rb_R2_v 
    yield d_rb_R3_v


def test_r2():
    from dynamics import d_rb_R1_v
    dt = 0.1
    
    state = d_rb_R1_v.integrate(state=np.array([0]),
                     commands=np.array([0]),
                     dt=dt)
    
