from . import EuclideanForce, EuclideanVel, Dynamics, np, contract
from geometry import Euclidean, ProductManifold, S1

def angle_from_S1(s):
    return  np.arctan2(s[1], s[0])
    
def S1_from_angle(angle):
    return np.array([np.cos(angle), np.sin(angle)])
    
class CircleVel(Dynamics):
    ''' Particle on S1 controlled in velocity. '''
    
    @contract(max_velocity='>0')
    def __init__(self, max_velocity):
        self.max_velocity = max_velocity
        self.R1 = EuclideanVel(ndim=1, max_velocity=[max_velocity])
        Dynamics.__init__(self,
                          pose_space=S1,
                          shape_space=None,
                          commands_spec=[(-1, 1)])
    
    def _integrate(self, state, commands, dt):
        # convert to angle
        theta = angle_from_S1(state)
        # integrate like in R
        theta2 = self.R1.integrate(theta, commands, dt)
        state2 = S1_from_angle(theta2) 
        return state2


class CircleForce(Dynamics):
    ''' Particle on S1 controlled in force. '''
    
    @contract(max_force='>0', mass='>0')
    def __init__(self, max_force, mass):
        self.R1 = EuclideanForce(ndim=1, max_force=[max_force], mass=mass)
        pose_space = ProductManifold((S1,
                                      Euclidean(1)))
        Dynamics.__init__(self,
                          pose_space=pose_space,
                          shape_space=None,
                          commands_spec=[(-1, 1)])
    
    def _integrate(self, state, commands, dt):
        position, velocity = state
        theta = angle_from_S1(position)
        theta2, velocity2 = self.R1.integrate((theta, velocity), commands, dt)
        position2 = S1_from_angle(theta2) 
        return (position2, velocity2)

        
