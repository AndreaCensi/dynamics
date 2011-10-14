from . import logger
from abc import ABCMeta, abstractmethod
from contracts import contract, new_contract
from geometry import ProductManifold
import traceback

new_contract('interval', 'tuple((number,x),(number,>x))')

class Dynamics:
    '''
        This is the interface for the various dynamics implemented.
        
        We make the following assumptions:
        
        1) The state space is the product of a pose space and a shape_space.
        2) No assumptions on shape_space, except that it is a DifferentiableManifold.
        3) It is assumed that state_space is homeomorphic to a subgroup of TSE(3). 
    
    
        In general, "state" is an opaque variable from outside.
        
        The function:
        
            state2 = dynamics.integrate(state, commands, dt)
            
        integrates the dynamics. Here state and state2 are opaque variables.
        
        The function:
        
            state = dynamics.pose2state(pose)
            
        creates a state from a pose (SE3), by embedding/projecting appropriately.
        The semantics is to get the closest state to the pose, with the dynamics
        "at rest"; for example, the velocity should be set to 0, and the
        shape space to a resting position.
        
        
        The function:
        
            pose, vel = dynamics.joint_state(state, joint=0)
            
        where SE3.belongs(pose) and se3.belongs(vel) returns the pos/vel
        of a joint for the given state.
            
    '''
    __metaclass__ = ABCMeta
    
    @contract(#commands_spec='list[K,>0](interval|list[>=2])',
              commands_spec='dict',
              pose_space='None|DifferentiableManifold',
              shape_space='None|DifferentiableManifold')
    def __init__(self, pose_space, shape_space, commands_spec):
        self._pose_space = pose_space
        self._shape_space = shape_space 
        self._commands_spec = commands_spec 
        
        if pose_space is not None and shape_space is not None:
            conf_space = pose_space.tangent_bundle()
            state_space = ProductManifold((conf_space, shape_space))
        elif pose_space is not None and shape_space is None:
            conf_space = pose_space.tangent_bundle()
            state_space = conf_space
        elif pose_space is None and shape_space is not None:
            state_space = shape_space
        else:
            raise ValueError('pose_space and shape_space cannot be both None.')
        self._state_space = state_space
        
    
    def check_commands(self, commands):
        ''' Raises an exception (InvalidCommands) if the commands are not valid. '''
        # TODO: not implemented
        pass

    def get_state_space(self):
        """ 
            Returns a Manifold instance describing the state space.
        
            If there is both a pose and a shape space, this is a ProductManifold. 
        """
        return self._state_space

    def get_pose_space(self):
        """ 
            Returns a DifferentiableManifold instance describing the pose space 
            (or None if we don't have one). 
        """
        return self._pose_space

    def get_shape_space(self):
        """ 
            Returns a DifferentiableManifold instance describing the shape space
            (or None if we don't have one). 
        """
        return self._shape_space

    def get_commands_spec(self):
        """ Returns the commands specification. """
        return self._commands_spec
    
    def get_commands_names(self):
        """ Returns the commands names. """
        return self._commands_names
        
    @contract(dt='>=0')
    def integrate(self, state, commands, dt):
        #self._state_space.belongs(state)    
        self.check_commands(commands)
        
        try:
            new_state = self._integrate(state, commands, dt)
        except:
            msg = 'Error while integrating.'
#            try:
#                msg += '\n   state: %s' % self._state_space.friendly(state)
#            except: # XXX
            msg += '\n   state: %s' % state.__repr__()
            msg += '\ncommands: %s' % commands.__repr__()
            msg += '\n%s' % traceback.format_exc()
            logger.error(msg)
            raise

        #self._state_space.belongs(new_state)
        return new_state
    
    @abstractmethod
    @contract(pose='SE3')
    def pose2state(self, pose):
        ''' Returns the state that best approximates the given pose (in SE3).''' 
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
    
    def __str__(self):
        return self.__class__.__name__

