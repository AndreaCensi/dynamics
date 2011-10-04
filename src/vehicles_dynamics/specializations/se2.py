from . import SimpleKinematics, contract
from geometry import (SE2, se2_from_linear_angular)

class SE2Dynamics(SimpleKinematics):
    
    @contract(max_linear_velocity='seq[2](>=0)',
              max_angular_velocity='>=0',)
    def __init__(self, max_linear_velocity, max_angular_velocity,
                 noise_drift=None, noise_mult=None):
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        spec = {
            'desc': 'Particle in SE2 controlled in velocity',
            'shape': [3],
            'format': ['C', 'C', 'C'],
            'range': [[-1, +1], [-1, +1], [-1, +1]],
            'names': ['vx', 'vy', 'angular velocity'],
            'rest': [0, 0],
            'extra': {'max_linear_velocity': max_linear_velocity,
                      'max_angular_velocity': max_angular_velocity,
                      'pose_space': 'SE2'}
        }
        SimpleKinematics.__init__(self,
                          pose_space=SE2,
                          commands_spec=spec,
                          noise_mult=noise_mult,
                          noise_drift=noise_drift)
    
    def compute_velocities(self, commands):
        linear = [ commands[0] * self.max_linear_velocity[0],
                    commands[1] * self.max_linear_velocity[1]]
        angular = commands[2] * self.max_angular_velocity
        vel = se2_from_linear_angular(linear, angular)
        return vel

