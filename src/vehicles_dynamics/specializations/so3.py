from . import SimpleKinematics, contract, np, SimpleDynamics
from geometry import (SO3, hat_map)


class SO3Vel(SimpleKinematics):
    
    @contract(max_angular_velocity='seq[3](>0)')
    def __init__(self, max_angular_velocity):
        self.max_angular_velocity = np.array(max_angular_velocity)
        SimpleKinematics.__init__(self,
                          pose_space=SO3,
                          commands_spec=[(-1, +1), (-1, +1), (-1, +1)])
    
    def compute_velocities(self, commands):
        return hat_map(self.max_angular_velocity * commands)

class SO3Force(SimpleDynamics):
    
    @contract(max_force='seq[3](>0)', mass='>0', damping='>=0')
    def __init__(self, max_force, mass, damping):
        self.max_forces = np.array(max_force)
        SimpleDynamics.__init__(self,
                          pose_space=SO3,
                          commands_spec=[(-1, +1), (-1, +1), (-1, +1)],
                          mass=mass, damping=damping)
    
    def compute_forces(self, commands):
        return hat_map(self.max_force * commands)
