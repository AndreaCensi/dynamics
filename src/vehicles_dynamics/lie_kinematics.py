from . import Dynamics, contract, np
from abc import abstractmethod
from geometry import SE3, se3
from geometry.yaml import to_yaml


class SimpleKinematics(Dynamics):
    ''' 
        A subclass for dynamics which have only a kinematic component.
        Here pose_space is assumed to be a subgroup of SE3,
        and the state is (pose, vel) where vel is a subalgebra of se3.
        
        The model includes stochastic noise, containing two terms:
        1) drift Gaussian noise - of fixed covariance.
        2) proportional to the velocities.
        
        More in detail, let algebra2vector(.) be the isomorphism from
        the algebra to R^n. 
        
        Suppose that *v0* are the "intentional" velocities (element of the
        Lie algebra). Then let: ::
        
            w0 = algebra2vector(v0)
            w = w0 + Normal(M0 + M1*|w|)
            v = vector2algebra(w)
            
        For simplicity, both M0 and M1 are assumed to be diagonal matrices.
        They are called, respectively, noise_drift and noise_mult below.
        
        configuration = (pose, vel)
        
    '''

    @contract(pose_space='DifferentiableManifold',
              noise_drift='None|seq[K](>=0)',
              noise_mult='None|seq[K](>=0)')
    def __init__(self, pose_space, commands_spec,
                 noise_drift=None, noise_mult=None):
        if not pose_space.embeddable_in(SE3):
            msg = 'I expect a subgroup of SE3.'
            raise ValueError(msg)
        algebra = pose_space.get_algebra()
        n = algebra.get_dimension()

        if noise_drift is None:
            noise_drift = np.zeros(n)

        if noise_mult is None:
            noise_mult = np.zeros(n)

        noise_drift = np.array(noise_drift)
        noise_mult = np.array(noise_mult)

        if noise_mult.size != n:
            msg = 'Wrong size for noise_mult: %s' % str(noise_mult)
            raise ValueError(msg)

        if noise_drift.size != n:
            msg = 'Wrong size for noise_mult: %s' % str(noise_mult)
            raise ValueError(msg)

        self.noise_drift = noise_drift
        self.noise_mult = noise_mult
        self.pose_space = pose_space

        conf_space = pose_space.tangent_bundle()

        Dynamics.__init__(self, state_space=conf_space,
                          commands_spec=commands_spec)

    def __repr__(self):
        return "LieKinematics(%s)" % self.pose_space

    def state_to_yaml(self, state):
        my_pose, my_vel = state
        self.pose_space.belongs(my_pose)
        self.pose_space.algebra.belongs(my_vel)
        pose = self.pose_space.embed_in(SE3, my_pose)
        vel = self.pose_space.algebra.embed_in(se3, my_vel)
        return to_yaml('TSE3', (pose, vel))

    @contract(pose='SE3')
    def pose2state(self, pose):
        ''' 
            Projects the pose to our subgroup, and sets the 
            velocity to zero. 
        '''
        my_pose = self.pose_space.project_from(SE3, pose)
        my_vel = self.pose_space.algebra.zero()
        assert my_pose.shape == my_vel.shape
        configuration = (my_pose, my_vel)
        return configuration

    @contract(state='tuple(*,*)', returns='tuple(SE3, se3)')
    def joint_state(self, state, joint=0):
        if joint != 0:
            raise ValueError('Must be implemented by subclasses.')
        my_pose, my_vel = state
        self.pose_space.belongs(my_pose)
        self.pose_space.algebra.belongs(my_vel)
        pose = self.pose_space.embed_in(SE3, my_pose)
        vel = self.pose_space.algebra.embed_in(se3, my_vel)
        configuration = (pose, vel)
        return configuration

    @abstractmethod
    def compute_velocities(self, commands):
        ''' 
            Subclasses should implement this and return an element 
            of the algebra. 
        '''

    def _integrate(self, state, commands, dt):
        pose1, unused_vel1 = state #@UnusedVariable
        # This is the noiseless part of the dynamics
        noiseless_vel = self.compute_velocities(commands)
        # Convert them to the vector representation
        # TODO: do not do any of this if the noise is 0
        algebra = self.pose_space.get_algebra()
        w0 = algebra.vector_from_algebra(noiseless_vel)
        # w = w0 + Normal(M0 + M1*|w|)
        # Variance of the noise
        variance = self.noise_drift + self.noise_mult * np.abs(w0)
        # Noise contribution
        wN = np.random.randn(variance.size) * np.sqrt(variance)
        w = w0 + wN
        # Convert back to algebra
        vel2 = algebra.algebra_from_vector(w)
        step = self.pose_space.group_from_algebra(vel2 * dt)
        pose2 = np.dot(pose1, step)
        config2 = pose2, vel2
        return config2

