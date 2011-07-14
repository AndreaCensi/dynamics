from . import SimpleDynamics, contract, SimpleKinematics
from geometry import SO2, hat_map_2d

    
class CircleVel(SimpleKinematics):
    ''' Particle on SO(2) controlled in velocity. '''
    
    @contract(max_velocity='>0')
    def __init__(self, max_velocity):
        SimpleKinematics.__init__(self,
                          pose_space=SO2,
                          commands_spec=[(-1, 1)])
        self.max_velocity = max_velocity    
    
    def compute_velocities(self, commands):
        omega = self.max_velocity * commands[0]
        return hat_map_2d(omega)


class CircleForce(SimpleDynamics):
    ''' Particle on SO(2) controlled in force. '''
    
    @contract(max_force='>0', mass='>0', damping='>=0')
    def __init__(self, max_force, mass, damping):
        SimpleDynamics.__init__(self,
                          pose_space=SO2,
                          commands_spec=[(-1, 1)],
                          mass=mass,
                          damping=damping)
        self.max_force = max_force
    

    def compute_forces(self, commands):
        force = self.max_force * commands[0]
        return hat_map_2d(force)
