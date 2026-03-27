import numpy as np


def sphere_sdf(x, y, z, radius):
    return np.sqrt(x**2 + y**2 + z**2) - radius

def cylinder_sdf(x, y, z, radius, height):
    return np.maximum(np.sqrt(x**2 + y**2) - radius, np.abs(z) - height / 2)