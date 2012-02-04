from . import logger
from abc import ABCMeta, abstractmethod
from contracts import contract, new_contract
import traceback
import geometry # For its contracts @UnusedImport
new_contract('interval', 'tuple((number,x),(number,>x))')


class Dynamics:
    '''
        This is the interface for the various dynamics implemented.
        
        We make the following assumptions:
        
        1) The state space is the product of a pose space and a shape_space.
        2) No assumptions on shape_space, except that it is a 
           DifferentiableManifold.
        3) It is assumed that state_space is homeomorphic 
           to a subgroup of TSE(3). 
    
    
        In general, "state" is an opaque variable from outside.
        
        The function:
        
            state2 = dynamics.integrate(state, commands, dt)
            
        integrates the dynamics. Here state and state2 are opaque variables.
        
        The function:
        
            state = dynamics.pose2state(pose)
            
        creates a state from a pose (SE3), by embedding/projecting 
        appropriately. The semantics is to get the closest state to the pose, 
        with the dynamics "at rest"; for example, the velocity should be set 
        to 0, and the shape space to a resting position.
        
        
        The function:
        
            pose, vel = dynamics.joint_state(state, joint=0)
            
        where SE3.belongs(pose) and se3.belongs(vel) returns the pos/vel
        of a joint for the given state.
            
    '''
    __metaclass__ = ABCMeta

    @contract(commands_spec='dict',
              state_space='DifferentiableManifold')
    def __init__(self, state_space, commands_spec):
        self._commands_spec = commands_spec
        self._state_space = state_space

    @contract(commands='array')
    def check_commands(self, commands):
        ''' 
            Raises an exception (InvalidCommands) if the commands 
            are not valid. 
        '''
        # TODO: not implemented
        pass

    @contract(returns='DifferentiableManifold')
    def get_state_space(self):
        """ 
            Returns a DifferentiableManifold instance 
            describing the state space.
        """
        return self._state_space

    @contract(returns='dict')
    def get_commands_spec(self):
        """ Returns the commands specification. """
        return self._commands_spec

    @contract(dt='>=0')
    def integrate(self, state, commands, dt):
        #self._state_space.belongs(state)    
        self.check_commands(commands)

        try:
            new_state = self._integrate(state, commands, dt)
        except:
            msg = 'Error while integrating.'
            try:
                msg += '\n   state: %s' % self._state_space.friendly(state)
            except: # XXX
                msg += '\n   state: %s' % state.__repr__()
            msg += '\ncommands: %s' % commands.__repr__()
            msg += '\n%s' % traceback.format_exc()
            logger.error(msg)
            raise

        #self._state_space.belongs(new_state)
        return new_state

    def __str__(self):
        return self.__class__.__name__

    # Interface that must be implemented

    @abstractmethod
    @contract(pose='SE3')
    def pose2state(self, pose):
        ''' 
            Returns the state that best approximates the given 
            pose (in SE3).
        '''
        pass

    @abstractmethod
    def _integrate(self, state, commands, dt):
        pass

    @abstractmethod
    def state_to_yaml(self, state):
        ''' Converts the state to a YAML representation.'''
        pass

    @abstractmethod
    @contract(returns='tuple(SE3, se3)')
    def joint_state(self, state, joint=0):
        pass

    @contract(returns='int,>=1')
    def num_joints(self):
        """ Returns the number of joints. At least 1. """
        return 1


