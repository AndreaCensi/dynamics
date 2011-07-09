from . import Dynamics

class DifferentialDrive(Dynamics):
    
    def __init__(self, max_angular_velocity):
        Dynamics.__init__(self, state_space=SE2)
        self.se2_dynamics = SE2Dynamics()
    
    def _check_commands(self, commands):
        commands = np.array(commands)
        check(commands, 'array[2],finite')
        
    def _integrate(self, state, commands, dt):
        driving = commands[0]
        steering = commands[1]
        commands = [driving, steering, 0]
        self.s
        
