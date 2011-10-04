from . import  np, contract, SimpleKinematics
from geometry import SE2, se2_from_linear_angular

class SimpleCar(SimpleKinematics):
    
    @contract(max_linear_velocity='>0',
              max_steering_angle='>0,<pi*0.5',
              L='>0')
    def __init__(self, max_linear_velocity, max_steering_angle, L):
        self.max_linear_velocity = max_linear_velocity
        self.max_steering_angle = max_steering_angle
        self.L = L
        spec = {
            'desc': 'Simple car dynamics',
            'shape': [2],
            'format': ['C', 'C'],
            'range': [[-1, +1], [-1, +1]],
            'names': ['driving velocity', 'steering angle'],
            'rest': [0, 0],
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


class ReedsSheepCar(SimpleCar):
    
    @contract(max_linear_velocity='>0',
              max_steering_angle='>0,<pi*0.5',
              L='>0')
    def __init__(self, max_linear_velocity, max_steering_angle, L):
        self.max_linear_velocity = max_linear_velocity
        self.max_steering_angle = max_steering_angle
        self.L = L
        spec = {
            'desc': 'Reeds-Sheep car dynamics (discretize velocity)',
            'shape': [2],
            'format': ['D', 'C'],
            'range': [[-1, +1], [-1, +1]],
            'names': ['driving velocity', 'steering angle'],
            'rest': [0, 0],
            'extra': {'max_linear_velocity': max_linear_velocity,
                      'max_steering_angle': max_steering_angle,
                      'L': L,
                      'pose_space': 'SE2'}
        }
        SimpleKinematics.__init__(self,
                          pose_space=SE2,
                          commands_spec=spec)

class DubinsCar(SimpleCar):
    
    @contract(max_linear_velocity='>0',
              max_steering_angle='>0,<pi*0.5',
              L='>0')
    def __init__(self, max_linear_velocity, max_steering_angle, L):
        self.max_linear_velocity = max_linear_velocity
        self.max_steering_angle = max_steering_angle
        self.L = L
        spec = {
            'desc': 'Dubins Car dynamics (no reverse)',
            'shape': [2],
            'format': ['D', 'C'],
            'range': [[0, +1],
                      [-1, +1]],
            'names': ['driving velocity', 'steering angle'],
            'rest': [0, 0],
            'extra': {'max_linear_velocity': max_linear_velocity,
                      'max_steering_angle': max_steering_angle,
                      'L': L, 'pose_space': 'SE2'}
        }
        SimpleKinematics.__init__(self,
                          pose_space=SE2,
                          commands_spec=spec)
