import numpy as np
from math import cos, sin, pi




def transform_frame_EulerXYZ(euler_angles, translation, point, degrees=True):
    '''
    We translate from frame A to frame B using homogenous coordinates.
    euler_angles describe the rotation of frame A relative to frame B.
    translation describes the offset of frame A relative to frame B. 
    point is the point in frame A that we want to convert to frame B.
    If degrees is set to true, the angles will be passed in as degrees
    '''

    R = get_rotation_matrix_EulerXYZ(euler_angles, degrees)

    # Total transformation matrix which includes the transformation matrix and the translation in homogenous coordinates
    T = np.eye(4, 4)
    T[:3, :3] = R
    T[:3, 3] = translation

    # Apply transformation on target point we want to transform into different coordinate system
    return np.dot(T, point)


def get_rotation_matrix_EulerXYZ(euler_angles, degrees=True):
    '''
    Get the rotation matrix from a set of Euler angles.
    '''

    if degrees:
        alpha = euler_angles[0] * pi / 180
        beta = euler_angles[1] * pi / 180
        gamma = euler_angles[2] * pi / 180
    else:
        alpha = euler_angles[0]
        beta = euler_angles[1]
        gamma = euler_angles[2]
    
    # Rotation matrix around x-axis - alpha is roll
    rx = [[1, 0, 0],
        [0, cos(alpha), sin(alpha)],
        [0, -sin(alpha), cos(alpha)]]

    # Rotation matrix around y-axis - beta is pitch
    ry = [[cos(beta), 0, -sin(beta)],
        [0, 1, 0], 
        [sin(beta), 0, cos(beta)]]

    # Rotation matrix around z-axis - gamma is yaw
    rz = [[cos(gamma), sin(gamma), 0],
        [-sin(gamma), cos(gamma), 0],
        [0, 0, 1]]

    # Multiplications such that the total rotation matrix R is R = rz * ry * rx
    R1 = np.matmul(ry, rx)
    R = np.matmul(rz, R1)
    return R

def get_transformation_matrix_about_arb_axis(centroid, angle, axis):
    '''
    This gets the transformation matrix to rotate a point around an arbitrary axis in space
    by a given amount. For this, we need a centroid of the frame we are rotating about (usually the object centroid)
    and the axis of the rotation in the frame that we are rotating in. 

    This is a bit longer sequence of operations that has a great explanation here: 
    https://www.eng.uc.edu/~beaucag/Classes/Properties/OptionalProjects/CoordinateTransformationCode/Rotate%20about%20an%20arbitrary%20axis%20(3%20dimensions).html
    '''
    
    
    t1 = np.array([[1, 0, 0, -centroid[0]],
                   [0, 1, 0, -centroid[1]],
                   [0, 0, 1, -centroid[2]],
                   [0, 0, 0, 1]])

    t1_inv = np.array([[1, 0, 0, centroid[0]],
                   [0, 1, 0, centroid[1]],
                   [0, 0, 1, centroid[2]],
                   [0, 0, 0, 1]])
    
    axis = np.dot(1/np.linalg.norm(axis), axis)
    a = axis[0]
    b = axis[1]
    c = axis[2]
    d = np.sqrt(b**2 + c**2)


    rx = np.array([[1, 0, 0, 0],
                   [0, c/d, -b/d, 0],
                   [0, b/d, c/d, 0],
                   [0, 0, 0, 1]])

    rx_inv = np.array([[1, 0, 0, 0],
                   [0, c/d, b/d, 0],
                   [0, -b/d, c/d, 0],
                   [0, 0, 0, 1]])

    ry = np.array([[d, 0, -a, 0],
                   [0, 1, 0, 0],
                   [a, 0, d, 0],
                   [0, 0, 0, 1]])
    
    ry_inv = np.array([[d, 0, a, 0],
                   [0, 1, 0, 0],
                   [-a, 0, d, 0],
                   [0, 0, 0, 1]])

    rz = np.array([[cos(angle), sin(angle), 0, 0],
                   [-sin(angle), cos(angle), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    T = np.dot(t1_inv, np.dot(rx_inv, np.dot(ry_inv, np.dot(rz, np.dot(ry, np.dot(rx, t1))))))
    return T