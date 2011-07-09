from . import SimpleKinematics, contract
from geometry import (SE2, se2_from_linear_angular)

class SE2Dynamics(SimpleKinematics):
    
    @contract(max_linear_velocity='seq[2](>0)',
              max_angular_velocity='>0',)
    def __init__(self, max_linear_velocity, max_angular_velocity):
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        SimpleKinematics.__init__(self,
                          pose_space=SE2,
                          commands_spec=[(-1, +1), (-1, +1), (-1, +1)])
    
    def compute_velocities(self, commands):
        linear = [ commands[0] * self.max_linear_velocity[0],
                    commands[1] * self.max_linear_velocity[1]]
        angular = commands[2] * self.max_angular_velocity
        vel = se2_from_linear_angular(linear, angular)
        return vel

