import numpy as np
import math


class Vector:

    def length(self, v):
        return np.sqrt(np.dot(v, v))

    def angle(self, v1, v2):
        x1, y1 = v1[0], v1[1]
        x2, y2 = v2[0], v2[1]
        return np.degrees(math.atan2(x1*y2-y1*x2, x1*x2+y1*y2))

    def rotate(self, v, deg):
        rad = math.radians(deg)
        x = v[0] * math.cos(rad) - v[1] * math.sin(rad)
        y = v[0] * math.sin(rad) + v[1] * math.cos(rad)
        return [x, y]

    def unit(self, v):
        return v / np.linalg.norm(v)
