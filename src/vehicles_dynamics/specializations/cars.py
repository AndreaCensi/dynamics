from . import np, contract, SimpleKinematics
from geometry import (R1, ProductManifold, SE2, se2_from_linear_angular,
    SE2_from_translation_angle, SE3_from_SE2, SE3)
from vehicles_dynamics import Dynamics


class SimpleCar(SimpleKinematics):

    @contract(max_linear_velocity='>0',
              max_steering_angle='>0,<pi*0.5',
              L='>0',
              axis_dist='>0')
    def __init__(self, max_linear_velocity, max_steering_angle, L,
                       axis_dist,
                       commands_format=['C', 'C'],
                       commands_range=[[-1, +1], [-1, +1]],
                       desc='Simple car dynamics'):
        self.max_linear_velocity = max_linear_velocity
        self.max_steering_angle = max_steering_angle
        self.L = L
        self.axis_dist = axis_dist
        spec = {
            'desc': desc,
            'shape': [2],
            'format': commands_format,
            'range': commands_range,
            'names': ['driving velocity',
                      'steering angle'],
            'default': [0, 0],
            'extra': {'max_linear_velocity': max_linear_velocity,
                      'max_steering_angle': max_steering_angle,
                      'L': L,
                      'pose_space': 'SE2'}
        }

        SimpleKinematics.__init__(self,
                          pose_space=SE2,
                          commands_spec=spec)

    def compute_velocities(self, commands):
        steering_angle = commands[1] * self.max_steering_angle
        linear_velocity = commands[0] * self.max_linear_velocity
        angular_velocity = np.tan(steering_angle) * linear_velocity / self.L
        vel = se2_from_linear_angular([linear_velocity, 0], angular_velocity)
        return vel


class CarWithWheels(Dynamics):

    def __init__(self, **kwargs):
        self.car = SimpleCar(**kwargs)

        state_space = ProductManifold((self.car.get_state_space(), R1))
        commands_spec = self.car.get_commands_spec()
        Dynamics.__init__(self, state_space=state_space,
                                commands_spec=commands_spec)

    def pose2state(self, pose):
        return self.compose_state(car_state=self.car.pose2state(pose),
                                  steering=0)

    def _integrate(self, state, commands, dt):
        car_state = self.car_state_from_big_state(state)
        car_state2 = self.car.integrate(car_state, commands, dt)
        steering = commands[1] * self.car.max_steering_angle
        return self.compose_state(car_state=car_state2, steering=steering)

    def state_to_yaml(self, state):
        car_state = self.car_state_from_big_state(state)
        steering = self.steering_from_big_state(state)
        return dict(car=self.car.state_to_yaml(car_state),
                    steering=steering)

    def car_state_from_big_state(self, state):
        return state['car']

    def steering_from_big_state(self, state):
        return state['steering']

    def compose_state(self, car_state, steering):
        return dict(car=car_state, steering=steering)

    @contract(returns='tuple(SE3, se3)')
    def joint_state(self, state, joint=0):
        car_state = self.car_state_from_big_state(state)
        steering = self.steering_from_big_state(state)

        car_position = self.car.joint_state(car_state, joint=0)

        if joint == 0:
            return car_position
        elif joint == 1:
            p = [self.car.axis_dist, self.car.L]
        elif joint == 2:
            p = [self.car.axis_dist, -self.car.L]
        else:
            raise ValueError('Invalid joint ID %r' % joint)

        pose, vel = car_position
        rel_pose = SE3_from_SE2(SE2_from_translation_angle(p, steering))
        wpose = SE3.multiply(pose, rel_pose)
        return wpose, vel # XXX: vel

    def num_joints(self):
        return 3


class ReedsSheepCar(SimpleCar):

    def __init__(self, **kwargs):
        SimpleCar.__init__(self,
                           desc='Reeds-Sheep car (discretize velocity)',
                           commands_format=['D', 'C'],
                           commands_range=[[-1, +1], [-1, +1]],
                           **kwargs)


class DubinsCar(SimpleCar):

    def __init__(self, **kwargs):
        SimpleCar.__init__(self,
                           desc='Dubins Car dynamics (no reverse)',
                           commands_format=['D', 'C'],
                           commands_range=[[0, +1], [-1, +1]],
                           **kwargs)


class ReedsSheepCarWithWheels(CarWithWheels):

    def __init__(self, **kwargs):
        CarWithWheels.__init__(self,
                           desc='Reeds-Sheep car (discretize velocity)',
                           commands_format=['D', 'C'],
                           commands_range=[[-1, +1], [-1, +1]],
                           **kwargs)


class DubinsCarWithWheels(CarWithWheels):

    def __init__(self, **kwargs):
        CarWithWheels.__init__(self,
                           desc='Dubins Car dynamics (no reverse)',
                           commands_format=['D', 'C'],
                           commands_range=[[0, +1], [-1, +1]],
                           **kwargs)
