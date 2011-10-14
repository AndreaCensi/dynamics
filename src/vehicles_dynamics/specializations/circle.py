from . import SimpleDynamics, contract, SimpleKinematics
from geometry import SO2, hat_map_2d

    
class CircleVel(SimpleKinematics):
    ''' Particle on SO(2) controlled in velocity. '''
    
    @contract(max_velocity='>0')
    def __init__(self, max_velocity):
        spec = {
            'desc': 'Particle on SO(2) controled in velocity.',
            'shape': [1],
            'format': ['C'],
            'range': [[-1, +1]],
            'names': ['velocity'],
            'default': [0],
            'extra': {'max_velocity': max_velocity,
                      'pose_space': 'SO2'}
        }
        
        SimpleKinematics.__init__(self,
                          pose_space=SO2,
                          commands_spec=spec)
        self.max_velocity = max_velocity    
    
    @contract(commands='array[1]')
    def compute_velocities(self, commands):
        omega = self.max_velocity * commands[0]
        return hat_map_2d(omega)


class CircleForce(SimpleDynamics):
    ''' Particle on SO(2) controlled in force. '''
    
    @contract(max_force='>0', mass='>0', damping='>=0')
    def __init__(self, max_force, mass, damping):
        spec = {
            'desc': 'Particle on SO(2) controled in force.',
            'shape': [1],
            'format': ['C'],
            'range': [[-1, +1]],
            'names': ['force'],
            'default': [0],
            'extra': {'max_force': max_force,
                      'mass': mass,
                      'damping': damping,
                      'pose_space': 'SO2'}
        }

        SimpleDynamics.__init__(self,
                          pose_space=SO2,
                          commands_spec=spec,
                          mass=mass,
                          damping=damping)
        self.max_force = max_force
    
    @contract(commands='array[1]')
    def compute_forces(self, commands):
        force = self.max_force * commands[0]
        return hat_map_2d(force)
