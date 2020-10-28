#! /usr/bin/env python3

"""
    # Utkarsh Kunwar
    # 07289641
    # utkarshk@kth.se
"""
import numpy as np
iter_methods = ["svd", "transpose", "pseudo"]
method = iter_methods[2]
current_params = None
# Gives the SCARA IK


def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """
    a0 = 0.07
    a1 = 0.3
    a2 = 0.35

    x = x - a0

    c2 = (x**2 + y**2 - a1**2 - a2**2) / (2.0 * a1 * a2)
    s2 = np.sqrt(1 - c2**2)

    k1 = a1 + a2 * c2
    k2 = a2 * s2
    q[0] = np.arctan2(y, x) - np.arctan2(k2, k1)
    q[1] = np.arctan2(s2, c2)
    q[2] = z

    return q

# Returns the DH Matrix.


def getDHDict(dh_line_list, joint_positions):
    dh_dict = dict({'alpha_i': [],
                    'a_i': [],
                    'd_i': [],
                    'theta_i': []
                    })
    keys = ['alpha_i', 'd_i', 'a_i', 'theta_i']
    for i, line in enumerate(dh_line_list):
        for j, key in enumerate(keys):
            if key == 'theta_i':
                dh_dict[key].append(joint_positions[i])
            else:
                value = line.split()[j + 1]
                value = float(value)
                if key == 'alpha_i':
                    value = np.deg2rad(value)
                dh_dict[key].append(value)

    return dh_dict


# Returns a list of Transformation Matrices.
def getTransformationMatrices(dh_dict):
    T = [np.array([
        [np.cos(dh_dict['theta_i'][i]), -np.sin(dh_dict['theta_i'][i]) * np.cos(dh_dict['alpha_i'][i]), np.sin(
            dh_dict['theta_i'][i]) * np.sin(dh_dict['alpha_i'][i]), dh_dict['a_i'][i] * np.cos(dh_dict['theta_i'][i])],
        [np.sin(dh_dict['theta_i'][i]), np.cos(dh_dict['theta_i'][i]) * np.cos(dh_dict['alpha_i'][i]), -np.cos(
            dh_dict['theta_i'][i]) * np.sin(dh_dict['alpha_i'][i]), dh_dict['a_i'][i] * np.sin(dh_dict['theta_i'][i])],
        [0.0, np.sin(dh_dict['alpha_i'][i]), np.cos(
            dh_dict['alpha_i'][i]), dh_dict['d_i'][i]],
        [0.0, 0.0, 0.0, 1.0]], dtype=np.float32) for i in range(len(dh_dict["alpha_i"]))]
    T.insert(0, np.array([[1, 0, 0, 0], [0, 1, 0, 0], [
             0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32))

    # All transformations wrt base frame
    for i in range(1, len(T)):
        T[i] = np.matmul(T[i - 1], T[i])

    return T


# Returns the rotation matrix from the transformation matrix
def getRotationMatrix(T):
    return T[:3, :3]


# Returns the cartesian coordinates from the transformation matrix
def getCoordinates(T):
    #     print("Coordinates = ", T[:3, -1].reshape(3))
    return T[:3, -1].reshape(3)


# Returns the Z-unit vector from the transformation matrix
def getRotationAxisUnitVector(T):
    return getRotationMatrix(T)[:, 2:].reshape(3)


# Forms the angular section of the Jacobian matrix
def formAngularSection(T):
    return getRotationAxisUnitVector(T)


# Gets relative position of frame n with respect to frame i
def getRelativePosition(Tn, Ti):
    n_coordinates = getCoordinates(Tn)
    i_coordinates = getCoordinates(Ti)
#     print("Relative position = ", n_coordinates - i_coordinates)
    return n_coordinates - i_coordinates


# Forms the liner section of the Jacobian matrix
def formLinearSection(Tn, Ti):
    return np.cross(getRotationAxisUnitVector(Ti), getRelativePosition(Tn, Ti))


# Forms the column for the Jacobian matrix for the ith joint
def formJacobianColumn(Tn, Ti):
    return np.stack([formLinearSection(Tn, Ti), formAngularSection(Ti)]).reshape(6, 1)


# Returns the Jacobian matrix from the list of transformation matrices.
def getJacobianMatrix(T_list):
    Tn = T_list[len(T_list) - 1]
    J = [formJacobianColumn(Tn, Ti) for Ti in T_list[:-1]]

    J = np.hstack(J)
    print("Jacobian: ", J)
    exit()
    return J


# Returns the orientation error part for the error
def getOrientError(Rd, Re):
    nd, sd, ad = Rd[:, 0], Rd[:, 1], Rd[:, 2]
    ne, se, ae = Re[:, 0], Re[:, 1], Rd[:, 2]

    eo = 0.5 * (np.cross(ne, nd) + np.cross(se, sd) + np.cross(ae, ad))
    return eo


# Returns the total error for the position and orientation
def getError(desired_params, current_params):
    linear_error = (desired_params[0] - current_params[0])
    orient_error = getOrientError(desired_params[1], current_params[1])

    error = np.array([linear_error, orient_error], dtype=np.float32).flatten()

    return error


# Returns the inverse of the Jacobian matrix.
def getJInverse(J):
    return np.matmul(np.linalg.inv(np.matmul(J.T, J)), J.T)

# Gives the norm of the error


def evalError(error):
    return np.linalg.norm(error)

# Gives the error and the change in position required to move the robot


def getUpdateStep(point, R, joint_positions, dh_params):
    lines = dh_params.split("\n")
    dh_dict = getDHDict(lines, joint_positions)
    T = getTransformationMatrices(dh_dict)

    J = getJacobianMatrix(T)
    desired_params = [point, R]
    global current_params
    current_params = [getCoordinates(T[-1]), getRotationMatrix(T[-1])]

    error = getError(desired_params, current_params)
    delta_theta = np.matmul(getJInverse(J), error)

    return np.array(error), delta_theta


# Gives the KUKA IK
def kuka_IK(point, R, joint_positions, error_threshold):
    # x = point[0]
    # y = point[1]
    # z = point[2]
    q = joint_positions  # it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    NOTE: DH params in the question statement in the table were wrong. Fixed by using d1 = 0.311 and d7 = 0.078.
    """

    dh_params = "1	90	0.311	0\n2	-90	0	0\n3	-90	0.4	0\n4	90	0	0\n5	90	0.39	0\n6	-90	0	0\n7	0	0.078	0"
    point = np.array(point, dtype=np.float32)
    q = np.array(q, dtype=np.float32)
    R = np.array(R, dtype=np.float32)

    threshold = error_threshold
    counter = 1
    MAX_STEPS = 3
    global current_params
    while True:
        new_error, delta_theta = getUpdateStep(point, R, q, dh_params)
        q = q + delta_theta
        if evalError(new_error) <= threshold:
            break

        counter += 1

    return q, current_params[0], current_params[1], new_error
