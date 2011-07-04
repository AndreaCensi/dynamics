from . import Dynamics, np, contract
import geometry
from geometry import ProductManifold

class EuclideanVel(Dynamics):
    ''' Particle in Euclidean space controlled in velocity. '''
    
    @contract(ndim='int,>0,N', max_velocity='seq[N](>0)')
    def __init__(self, ndim, max_velocity):
        self.ndim = ndim
        self.max_velocity = max_velocity
        pose_space = geometry.Euclidean(ndim)
        Dynamics.__init__(self, pose_space=pose_space, commands_spec=[(-1, 1)] * ndim)
    
    def _integrate(self, state, commands, dt):
        return state + commands * dt


class EuclideanForce(Dynamics):
    ''' Particle in Euclidean space controlled in force. '''
    
    @contract(ndim='int,>0,N', max_force='seq[N](>0)', mass='>0')
    def __init__(self, ndim, max_force, mass):
        self.ndim = ndim
        self.max_force = max_force
        self.mass = mass
        pose_space = ProductManifold(geometry.Euclidean(ndim), geometry.Euclidean(ndim))
        Dynamics.__init__(self, pose_space=pose_space, commands_spec=[(-1, 1)] * ndim)
    
    def _integrate(self, state, commands, dt):
        position, velocity = state
        forces = np.array(commands)
        position2 = position + velocity * dt + 0.5 * velocity * dt * dt 
        velocity2 = velocity + forces * dt 
        return (position2, velocity2)

        
