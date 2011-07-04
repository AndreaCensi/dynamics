from contracts import contract, check, new_contract
from abc import ABCMeta, abstractmethod

from geometry import ProductManifold
from . import DynamicsException


new_contract('interval', 'tuple((number,x),(number,>x))')

class Dynamics:
    __metaclass__ = ABCMeta
    
    @contract(commands_spec='list[K,>0](interval|list[>=2])',
              commands_names='None|list[K](str)',
              pose_space='None|DifferentiableManifold',
              shape_space='None|DifferentiableManifold')
    def __init__(self, pose_space, shape_space, commands_spec, commands_names=None):
        self.commands_spec = commands_spec
        if pose_space is not None and shape_space is not None:
            state_space = ProductManifold((pose_space, shape_space))
        elif pose_space is not None and shape_space is None:
            state_space = pose_space
        elif pose_space is None and shape_space is not None:
            state_space = shape_space
        else:
            raise ValueError('pose_space and shape_space cannot be both None.')
        
        self.__state_space = state_space
        self.__pose_space = pose_space
        self.__shape_space = shape_space
    
    def check_commands(self, commands):
        ''' Raises an exception (InvalidCommands) if the commands are not valid. '''
        pass
    
    def state_space(self):
        """ 
            Returns a Manifold instance describing the state space.
        
            If there is both a pose and a shape space, this is a ProductManifold. 
        """
        return self.__state_space

    def pose_space(self):
        """ 
            Returns a Manifold instance describing the pose space 
            (or None if we don't have one). 
        """
        return self.__pose_space

    def shape_space(self):
        """ 
            Returns a Manifold instance describing the shape space
            (or None if we don't have one). 
        """
        return self.__shape_space

    def get_commands_spec(self):
        """ Returns a commands spec """
        pass
        
    @contract(dt='>0')
    def integrate(self, state, commands, dt):
        self.__state_space.belongs(state)    
        self.check_commands(commands)
        
        try:
            new_state = self._integrate(state, commands, dt)
        except Exception as e:
            msg = 'Error while integrating.'
            msg += '\n   state: %s' % state
            msg += '\ncommands: %s' % commands
            msg += '\n%s' % e
            raise DynamicsException(msg)
        
        self.__state_space.belongs(new_state)
        return new_state
    
    @abstractmethod
    def _integrate(self, state, commands, dt):
        pass

    def compute_relative_pose(self, state, relative_pose, joint=0):
        pass

class DifferentialDrive(Dynamics):
    
    def __init__(self):
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
        
 
