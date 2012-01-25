from . import SimpleKinematics, contract
from geometry import (SE2, se2_from_linear_angular)
import numpy as np


class DifferentialDrive(SimpleKinematics):

    @contract(max_linear_velocity='>=0',
              max_angular_velocity='>=0',)
    def __init__(self, max_linear_velocity, max_angular_velocity):
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        spec = {
            'desc': 'Differential drive dynamics',
            'shape': [2],
            'format': ['C', 'C'],
            'range': [[-1, +1], [-1, +1]],
            'names': ['angular velocity', 'linear velocity'],
            'default': [0, 0],
            'extra': {'max_linear_velocity': max_linear_velocity,
                      'max_angular_velocity': max_angular_velocity,
                      'pose_space': 'SE2'}
        }
        SimpleKinematics.__init__(self,
                          pose_space=SE2,
                          commands_spec=spec)

    def compute_velocities(self, commands):
        linear = np.array([commands[1] * self.max_linear_velocity, 0])
        angular = commands[0] * self.max_angular_velocity
        vel = se2_from_linear_angular(linear, angular)
        return vel

