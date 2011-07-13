from . import Dynamics, contract, np
from geometry import  SE3, se3
from abc import abstractmethod

class SimpleKinematics(Dynamics):
    ''' 
        A subclass for dynamics which have only a kinematic component.
        Here pose_space is assumed to be a subgroup of SE3,
        and the state is (pose, vel) where vel is a subalgebra of se3.
    '''
    
    def __init__(self, pose_space, commands_spec):
        self.pose_space = pose_space
        shape_space = None
        Dynamics.__init__(self, pose_space, shape_space, commands_spec) 
    
    def __repr__(self):
        return "LieKinematics(%s)" % self.pose_space
    
    @contract(pose='SE3')
    def pose2state(self, pose):
        ''' Projects the pose to our subgroup, and sets the velocity to zero. '''
        my_pose = self.pose_space.project_from(SE3, pose)
        my_vel = self.pose_space.algebra.zero()
        assert my_pose.shape == my_vel.shape 
        state = (my_pose, my_vel)
        return state

    @contract(returns='tuple(SE3, se3)')
    def joint_state(self, state, joint=0):
        my_pose, my_vel = state
        assert my_pose.shape == my_vel.shape
        pose = self.pose_space.embed_in(SE3, my_pose)
        vel = self.pose_space.algebra.embed_in(se3, my_vel) 
        return pose, vel
 
    @abstractmethod
    def compute_velocities(self, commands):
        ''' Subclasses should implement this, return an element of the algebra. '''

    def _integrate(self, state, commands, dt):
        pose1, unused_vel1 = state #@UnusedVariable
        vel2 = self.compute_velocities(commands)
        step = self.pose_space.group_from_algebra(vel2 * dt)
        pose2 = np.dot(pose1, step)
        return pose2, vel2
 
class SimpleDynamics(SimpleKinematics):

    @contract(mass='>0')
    def __init__(self, pose_space, commands_spec, mass):
        SimpleKinematics.__init__(self, pose_space, commands_spec) 
        self.mass = mass

    def __repr__(self):
        return "LieDynamics(%s)" % self.pose_space
        
    def compute_velocities(self, commands):
        raise ValueError('Not needed!.')

    @abstractmethod
    def compute_forces(self, commands):
        pass

    def _integrate(self, state, commands, dt):
        pose1, vel1 = state #@UnusedVariable
        forces = self.compute_forces(commands)
        #self.pose_space.algebra.belongs(forces)
        # TODO: this is not in closed form
        # TODO: make smaller steps
        acc = forces / self.mass
        vel2 = vel1 + dt * acc
        midvel = 0.5 * (vel1 + vel2)
        step = self.pose_space.group_from_algebra(midvel * dt)
        pose2 = np.dot(pose1, step)
        return pose2, vel2
 
