from . import OneJointDynamics, np, contract
from geometry import (SE2, se2_from_linear_angular, SE2_from_se2)

class SE2Dynamics(OneJointDynamics):
    
    @contract(max_linear_velocity='seq[2](>0)',
              max_angular_velocity='>0',)
    def __init__(self, max_linear_velocity, max_angular_velocity):
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        OneJointDynamics.__init__(self,
                          pose_space=SE2,
                          shape_space=None,
                          commands_spec=[(-1, +1), (-1, +1), (-1, +1)])
    
    def _integrate(self, state, commands, dt):
        linear = [ commands[0] * self.max_linear_velocity[0],
                    commands[1] * self.max_linear_velocity[1]]
        angular = commands[2] * self.max_angular_velocity
        vel = se2_from_linear_angular(linear, angular)
        step = SE2_from_se2(vel * dt)
        return np.dot(state, step)

