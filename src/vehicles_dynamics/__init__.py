__version__ = '0.5'

import numpy as np
from contracts import contract
import logging

logging.basicConfig();
logger = logging.getLogger("vehicles_dynamics")
logger.setLevel(logging.DEBUG)


from .exceptions import *
from .interface import *
from .simple_kinematics import *
from .specializations import *
