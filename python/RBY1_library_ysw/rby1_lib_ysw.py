"""Total Library"""
"""Just type "import rby1_lib_ysw as ysw" in other code files"""

from robot_info import *
from default_lib import *
from dynamics_lib import *
from gripper_lib import *

import logging
import sys


def setupLogging(level=logging.INFO):
    """logging이 매번 보이게 설정"""
    logging.basicConfig(
        level=level,
        # format="[%(levelname)s] %(message)s",
        format="%(asctime)s [%(levelname)s] %(message)s",
        stream=sys.stdout,
        force=True,
    )