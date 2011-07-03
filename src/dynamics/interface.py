from contracts import contract, ContractException
from abc import ABCMeta, abstractmethod
from geometry import *

class DynamicsException(Exception): pass

class InvalidState(DynamicsException): 
    def __init__(self, dynamics, state):
        self.dynamics = dynamics
        self.state = state
        msg = 'Invalid state %r for %s.' % (state, dynamics)
        DynamicsException.__init__(msg) 
    
class InvalidCommands(DynamicsException): 
    def __init__(self, dynamics, commands):
        self.dynamics = dynamics
        self.commands = commands
        msg = 'Invalid commands %r for %s.' % (commands, dynamics)
        DynamicsException.__init__(msg) 


class Dynamics:
    __metaclass__ = ABCMeta
    
    def __init__(self, state_space):
        self.state_space = state_space
    
    def state_space(self):
        """ Returns a string describing the state space. """
        return self.state_space

    def get_commands_spec(self):
        """ Returns a commands spec """
        pass
    
    @abstractmethod
    def _integrate(self, state, commands, dt):
        pass
    
    def integrate(self, state, commands, dt):
        try:
            return self._integrate(state, commands, dt)
        except ContractException as e:
            msg = 'Error while integrating.'
            msg += '\n   state: %s' % state
            msg += '\ncommands: %s' % commands
            msg += '\n%s' % e
            raise DynamicsException(self)
    
class EUVel(Dynamics):
    
    @contract(ndim='int,>0')
    def __init__(self, ndim):
        self.ndim = ndim
        state_space = 'R%d' % ndim
        Dynamics.__init__(self, state_space=state_space)
    
    def _integrate(self, state, commands, dt):
        if state.size != self.ndim:
            raise InvalidState(self, state)
        
        return state + commands * dt


class SE2Dynamics(Dynamics):
    
    def __init__(self):
        Dynamics.__init__(self, state_space=SE2)
    
    def _integrate(self, state, commands, dt):
        vel = se2_from_linear_angular([commands[0], commands[1]], commands[2])
        step = SE2_from_se2(vel * dt)
        return np.dot(state, step)

    
rb_SE2_v = SE2Dynamics()

#    
#class RigidBodyForce(Dynamics):
#    
#    @contract(ndim='int,>0,D', inertia='array[DxD]')
#    def __init__(self, ndim, inertia):
#        configuration_space = 'R%d' % ndim
#        Dynamics.__init__(configuration_space)
    
d_rb_R1_v = EUVel(1)
d_rb_R2_v = EUVel(2)
d_rb_R3_v = EUVel(3)

