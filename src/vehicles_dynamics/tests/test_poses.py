from geometry import SE2, SE2_from_translation_angle, SE3_from_SE2

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
            
            (([0, 0], np.radians(90)), [M, Z, Z], ([0, dt], np.radians(90))),
            (([0, 0], np.radians(90)), [Z, M, Z], ([-dt, 0], np.radians(90)))
            
            # TODO: add some more tests with non-zero initial rotation
        ]
        
        for initial, commands, final in tests:
            start_pose = SE2_from_translation_angle(*initial)
            final_pose = SE2_from_translation_angle(*final)
            
            start_state = dynamics.pose2state(SE3_from_SE2(start_pose))
            final_state = dynamics.pose2state(SE3_from_SE2(final_pose))
            commands = np.array(commands)

            print('%s -> [%s] -> %s' % 
                  (SE2.friendly(start_pose), commands, SE2.friendly(final_pose)))
            
            actual, dummy = dynamics.integrate(start_state, +commands, dt)              
            SE2.assert_close(actual, final_pose)
            
            start2, dummy = dynamics.integrate(final_state, -commands, dt)
            SE2.assert_close(start_pose, start2)
            
        
