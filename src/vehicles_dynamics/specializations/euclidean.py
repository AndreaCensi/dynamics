from . import SimpleDynamics, SimpleKinematics, np, contract
from geometry import Tran2, Tran3, Tran1

class EuclideanVel(SimpleKinematics):
    ''' Particle in Euclidean space controlled in velocity. '''
    
    @contract(ndim='int,>0,N', max_velocity='seq[N](>0)')
    def __init__(self, ndim, max_velocity):
        self.ndim = ndim
        self.max_velocity = np.array(max_velocity)
        pose_space = {1: Tran1, 2: Tran2, 3: Tran3}[ndim]
        SimpleKinematics.__init__(self, pose_space,
                            commands_spec=[(-1, 1)] * ndim)
    
    def compute_velocities(self, commands):
        vel = self.max_velocity * np.array(commands)
        return self.pose_space.algebra.from_vector(vel)

class EuclideanForce(SimpleDynamics):
    ''' Particle in Euclidean space controlled in force. '''
    
    @contract(ndim='int,>0,N', max_force='seq[N](>0)', mass='>0')
    def __init__(self, ndim, max_force, mass, damping):
        self.ndim = ndim
        self.max_force = max_force
        self.mass = mass
        pose_space = {1: Tran1, 2: Tran2, 3: Tran3}[ndim]
        SimpleDynamics.__init__(self,
                          pose_space=pose_space,
                          commands_spec=[(-1, 1)] * ndim,
                          mass=mass,
                          damping=damping)
    
    def compute_forces(self, commands):
        f = self.max_force * np.array(commands)
        return self.pose_space.algebra.from_vector(f)
