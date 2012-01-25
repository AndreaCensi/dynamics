from . import contract, np, SimpleKinematics
from abc import abstractmethod


class SimpleDynamics(SimpleKinematics):
    # TODO: add noise

    @contract(mass='>0')
    def __init__(self, pose_space, commands_spec, mass, damping):
        SimpleKinematics.__init__(self, pose_space, commands_spec)
        self.mass = mass
        self.damping = damping

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
        # TODO: make smaller step
        acc = (forces - vel1 * self.damping) / self.mass # XXX: like this?
        vel2 = vel1 + dt * acc
        midvel = 0.5 * (vel1 + vel2)
        step = self.pose_space.group_from_algebra(midvel * dt)
        pose2 = np.dot(pose1, step)
        return pose2, vel2

