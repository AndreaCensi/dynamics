from geometry import SE2, SE2_from_translation_angle

from numpy import radians
import unittest
import numpy as np

class PoseTest(unittest.TestCase):
    
    def test_poses_SE2(self):
        from vehicles_dynamics import SE2Dynamics

        dynamics = SE2Dynamics(max_linear_velocity=[1, 1],
                               max_angular_velocity=1)
        
        dt = 0.1
        
        M = 1  # max
        Z = 0  # zero 
        m = -1 # min
        
        tests = [
            # format   (  (start_xy, start_theta),  commands, (final_xy, final_theta))
            (([0, 0], 0), [Z, Z, Z], ([0, 0], 0)),
            (([1, 2], 3), [Z, Z, Z], ([1, 2], 3)),
            
            (([0, 0], 0), [M, Z, Z], ([dt, 0], 0)),
            
            (([0, 0], 0), [m, Z, Z], ([-dt, 0], 0)),
            (([0, 0], 0), [Z, M, Z], ([0, dt], 0)),
            (([0, 0], 0), [Z, m, Z], ([0, -dt], 0)),
            (([0, 0], 0), [Z, Z, M], ([0, 0], dt)),
            (([0, 0], 0), [Z, Z, m], ([0, 0], -dt)),
            
            (([0, 0], radians(90)), [M, Z, Z], ([0, dt], radians(90))),
            (([0, 0], radians(90)), [Z, M, Z], ([-dt, 0], radians(90)))
            
            # TODO: add some more tests with non-zero initial rotation
        ]
        
        for initial, commands, final in tests:
            start_state = SE2_from_translation_angle(*initial)
            final_state = SE2_from_translation_angle(*final)
            print('%s -> [%s] -> %s' % 
                  (SE2.friendly(start_state), commands, SE2.friendly(final_state)))
            commands = np.array(commands)
            actual = dynamics.integrate(start_state, +commands, dt)              
            SE2.assert_close(actual, final_state)
            start2 = dynamics.integrate(final_state, -commands, dt)
            SE2.assert_close(start_state, start2)
            
        
