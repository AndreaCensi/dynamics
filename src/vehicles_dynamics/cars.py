from . import Dynamics, np, contract
from geometry import (SE2, se2_from_linear_angular, SE2_from_se2)
from vehicles_dynamics.interface import OneJointDynamics

class SimpleCar(OneJointDynamics):
    
    @contract(max_linear_velocity='>0',
              max_steering_angle='>0,<pi*0.5',
              L='>0')
    def __init__(self, max_linear_velocity, max_steering_angle, L):
        self.max_linear_velocity = max_linear_velocity
        self.max_steering_angle = max_steering_angle
        self.L = L
        OneJointDynamics.__init__(self,
                          pose_space=SE2,
                          shape_space=None,
                          commands_spec=[(-1, +1), (-1, +1)])
    
    def _integrate(self, state, commands, dt):
        steering_angle = commands[1] * self.max_steering_angle
        linear_velocity = commands[0] * self.max_linear_velocity
        angular_velocity = np.tan(steering_angle) * linear_velocity / self.L
        vel = se2_from_linear_angular([linear_velocity, 0], angular_velocity)
        step = SE2_from_se2(vel * dt)
        return np.dot(state, step)


class ReedsSheepCar(SimpleCar):
    
    @contract(max_linear_velocity='>0',
              max_steering_angle='>0,<pi*0.5',
              L='>0')
    def __init__(self, max_linear_velocity, max_steering_angle, L):
        self.max_linear_velocity = max_linear_velocity
        self.max_steering_angle = max_steering_angle
        self.L = L
        Dynamics.__init__(self,
                          pose_space=SE2,
                          shape_space=None,
                          commands_spec=[[-1, 0, +1], (-1, +1)])

class DubinsCar(SimpleCar):
    
    @contract(max_linear_velocity='>0',
              max_steering_angle='>0,<pi*0.5',
              L='>0')
    def __init__(self, max_linear_velocity, max_steering_angle, L):
        self.max_linear_velocity = max_linear_velocity
        self.max_steering_angle = max_steering_angle
        self.L = L
        Dynamics.__init__(self,
                          pose_space=SE2,
                          shape_space=None,
                          commands_spec=[[0, +1], (-1, +1)])
