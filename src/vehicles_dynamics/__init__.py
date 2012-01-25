__version__ = '1.1'

import numpy as np
from contracts import contract
import logging

logging.basicConfig()
logger = logging.getLogger("vehicles_dynamics")
logger.setLevel(logging.DEBUG)


from .exceptions import *
from .interface import *
from .lie_kinematics import *
from .lie_dynamics import *
from .specializations import *
