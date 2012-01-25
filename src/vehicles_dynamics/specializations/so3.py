from . import SimpleKinematics, contract, np, SimpleDynamics
from geometry import (SO3, hat_map)


class SO3Vel(SimpleKinematics):

    @contract(max_angular_velocity='seq[3](>0)')
    def __init__(self, max_angular_velocity):
        self.max_angular_velocity = np.array(max_angular_velocity)
        spec = {
            'desc': 'Particle in SO3 controlled in velocity.',
            'shape': [3],
            'format': ['C', 'C', 'C'],
            'range': [[-1, +1], [-1, +1], [-1, +1]],
            'names': ['w1', 'w2', 'w3'],
            'default': [0, 0, 0],
            'extra': {'max_angular_velocity': max_angular_velocity,
                      'pose_space': 'SO3'}
        }

        SimpleKinematics.__init__(self, pose_space=SO3, commands_spec=spec)

    def compute_velocities(self, commands):
        return hat_map(self.max_angular_velocity * commands)


class SO3Force(SimpleDynamics):

    @contract(max_force='seq[3](>0)', mass='>0', damping='>=0')
    def __init__(self, max_force, mass, damping):
        self.max_force = np.array(max_force)
        spec = {
            'desc': 'Particle in SO3 controlled in force.',
            'shape': [3],
            'format': ['C', 'C', 'C'],
            'range': [[-1, +1], [-1, +1], [-1, +1]],
            'names': ['t1', 't2', 't3'],
            'default': [0, 0, 0],
            'extra': {'max_force': max_force,
                      'mass': mass,
                      'damping': damping,
                      'pose_space': 'SO3'}
        }
        SimpleDynamics.__init__(self,
                          pose_space=SO3,
                          commands_spec=spec,
                          mass=mass, damping=damping)

    def compute_forces(self, commands):
        return hat_map(self.max_force * commands)
