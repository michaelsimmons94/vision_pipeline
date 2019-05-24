#!/usr/bin/env python
import numpy as np
from scipy.spatial.transform import Rotation as R

r = R.from_euler('xyz', [[-150, 0, -90]], degrees=True)

print(r.as_quat())

