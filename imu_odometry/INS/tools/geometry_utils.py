import numpy as np
from .geometry_helpers import euler2quat, quat2euler, mat2euler, euler2mat

def rotateOutput(x, roll, pitch, yaw, axes='sxyz'):
    """Given a state vector and a rotation, returns a rotated state vector

    :param x: State vector [x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw]
    :param rotation: Euler angle rotation [roll, pitch, yaw]
    :param axes : str, optional
        Axis specification; one of 24 axis sequences as string or encoded
        tuple - e.g. ``sxyz`` (the default).

    :return Rotated State vector [x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw]
    """

    rotationMat = euler2mat(roll, pitch, yaw, axes)

    x_rotated = np.zeros((9))

    # rotate position
    x_rotated[0:3] = np.dot(rotationMat, x[0:3])

    # rotate velocity
    x_rotated[3:6] = np.dot(rotationMat, x[3:6])

    orientation = euler2mat(x[6], x[7], x[8], 'sxyz')
    # Apply rotation to orientation
    orientation = np.dot(rotationMat, orientation)

    r, p, y = mat2euler(orientation, 'sxyz')
    x_rotated[6:9] = [r, p, y]

    return x_rotated 