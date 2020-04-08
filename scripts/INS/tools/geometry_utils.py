import numpy as np
from .geometry_helpers import euler2quat, quat2euler, mat2euler, euler2mat

def rotateOutput(x, roll, pitch, yaw):
    """Given a state matrix and a rotation, returns a rotated state matrix

    :param x: State matrix [x, y, z, vel_x, vel_y, vel_z, roll, pitch, yaw]
    :param rotation: Euler angle rotation [roll, pitch, yaw]

    :return Rotated State matrix [x, y, z, roll, pitch, yaw, quart_w, quart_x, quart_y, quart_z]
    """

    rotationMat = euler2mat(roll, pitch, yaw, 'sxyz')

    x_rotated = np.zeros((x.shape[0], 10))

    for line in range(len(x)):
        # rotate position
        x_rotated[line,0:3] = np.dot(rotationMat, x[line,0:3])

        orientation = euler2mat(x[line,6], x[line,7], x[line,8], 'sxyz')
        # Apply rotation to orientation
        orientation = np.dot(rotationMat, orientation)

        r, p, y = mat2euler(orientation, 'sxyz')
        # Calculate quaternion
        w, qx, qy, qz = euler2quat(r, p, y, 'sxyz')

        x_rotated[line,3:10] = [r, p, y, w, qx, qy, qz]

    return x_rotated 