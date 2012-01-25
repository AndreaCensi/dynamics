from . import SimpleKinematics, contract
from geometry import (SE2, se2_from_linear_angular)


class SE2Forward(SimpleKinematics):

    @contract(linear_velocity='>0',
              max_angular_velocity='>0')
    def __init__(self, linear_velocity, max_angular_velocity,
                 noise_mult=None, noise_drift=None):
        self.linear_velocity = linear_velocity
        self.max_angular_velocity = max_angular_velocity
        spec = {
            'desc': 'Kinematic fly: fixed forward velocity',
            'shape': [1],
            'format': ['C'],
            'range': [[-1, +1]],
            'names': ['angular velocity'],
            'default': [0],
            'extra': {'linear_velocity': linear_velocity,
                      'max_angular_velocity': max_angular_velocity,
                      'pose_space': 'SE2'}
        }

        SimpleKinematics.__init__(self,
                          pose_space=SE2,
                          commands_spec=spec,
                          noise_mult=noise_mult,
                          noise_drift=noise_drift)

    def compute_velocities(self, commands):
        linear = [self.linear_velocity, 0]
        angular = float(commands[0]) * self.max_angular_velocity
        vel = se2_from_linear_angular(linear, float(angular))
        return vel

