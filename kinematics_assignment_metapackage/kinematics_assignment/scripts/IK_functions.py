#! /usr/bin/env python3
import numpy as np
import time

"""
    # {student full name} Sumit Patidar
    # {student email} patidar@kth.se
"""
iter_methods = ["svd", "transpose", "pseudo"]
method = iter_methods[2]


def scara_IK(point):
    def scara_IK(point):
        x = point[0]
        y = point[1]
        z = point[2]
        q = [0.0, 0.0, 0.0]

        """
    Fill in your IK solution here and return the three joint values in q
    """
        # define the constants
        a0 = 0.07
        a1 = 0.3
        a2 = 0.35

        # transform the points w.r.t to base frame
        x = x - a0

        # computing cos(theta2)
        c2 = (np.square(x) + np.square(y) -
              np.square(a1) - np.square(a2))/(2*a1*a2)

        # condition check - uncomment if needed
        # if np.absolute(c2) > 1:
        # print('No soln exist as cos(theta2) is out of range.')

        s2 = np.sqrt(1 - np.square(c2))

        # compute theta2
        for j in range(2):
            theta2 = np.arctan2(s2, c2)

        # computing theta1 using theta2
        for i in range(2):
            theta1 = np.arctan2(y, x) - np.arctan2(a2*s2, (a1 + a2*c2))

        # assign joint parametes and return
        q[0] = theta1
        q[1] = theta2
        q[2] = z
        return q


def GetJacobianInverse_SVD(jacob):
    u, s, vh = np.linalg.svd(jacob, full_matrices=True)
    s_inv = np.zeros(s.shape)
    for i in range(s.size):
        if s[i] != 0:
            s_inv[i] = 1/s[i]

    s_inv = np.diag(s_inv)  # make s_inv diagonal matrix
    c = np.zeros(6)  # add extra row to adjust the dimension
    s_inv = np.append(s_inv, c)

    s_inv = s_inv.reshape((7, 6))
    jacob_inv = np.matmul(vh.T, s_inv)
    jacob_inv = np.matmul(jacob_inv, u.T)
    return jacob_inv


def GetOrientationError(R_e, R_d):

    orient_error = 0.5*(np.cross(R_e[:, 0], R_d[:, 0]) + np.cross(
        R_e[:, 1], R_d[:, 1]) + np.cross(R_e[:, 2], R_d[:, 2]))
    return orient_error


def GetJacobianInverse_pseduo(J):
    a = np.matmul(J, J.T)
    a_inv = np.linalg.inv(a)
    return np.matmul(J.T, a_inv)


def get_alpha(error, jacob):
    x = np.matmul(jacob, jacob.T)
    x = np.matmul(x, error)
    x = x.reshape(6)
    error = error.reshape(6)
    num = np.inner(error, x)
    den = np.inner(x, x)
    alpha = num/den
    # print('alpha: ', alpha)
    return alpha


def kuka_IK(point, R, joint_positions, er_threshold):
    start_time = time.time()
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions  # it must contain 7 elements
    R = np.array(R)
    """
    Fill in your IK solution here and return the seven joint values in q
    """

    # define constants
    L = 0.4
    M = 0.39
    # define DH parameters - do they need to be modified ?
    alpha = np.array([np.pi/2, -np.pi/2, -np.pi/2,
                      np.pi/2, np.pi/2, -np.pi/2, 0.0])
    d = np.array([0.311, 0.0, L, 0.0, M, 0.0, 0.078])
    a = np.zeros(7)
    links = 7

    error_threshold = er_threshold
    max_iterations = 10
    current_iteration = 0

    # set desired position
    desired_pos = np.zeros((3, 1))
    desired_pos[0:3, 0] = point  # check if the point format is correct ?
    # desired_angles = GetEulerAngles2(R)
    # desired_angles = rotationMatrixToEulerAngles(R)
    # desired_pos[3:6,0] = desired_angles

    while True:
        current_iteration = current_iteration + 1
        # print('Iteration: ', current_iteration)

        # carry out transformation
        T = []
        T_trans = np.identity(4)
        for i in range(links):
            T_temp = np.array([[np.cos(q[i]), -np.sin(q[i])*np.cos(alpha[i]), np.sin(q[i])*np.sin(alpha[i]), a[i]*np.cos(q[i])], [np.sin(q[i]), np.cos(q[i])*np.cos(alpha[i]), -np.cos(q[i])*np.sin(alpha[i]), a[i]*np.sin(q[i])],
                               [0.0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]], [0.0, 0.0, 0.0, 1]])
            # print('t', T_temp)
            T_trans = np.matmul(T_trans, T_temp)
            # print('T trans',T_trans)
            T.append(T_trans)
            # print('T',T[i])

        # get p vectors - 8 vectors of size 3
        p = []
        p.append(np.array([[0], [0], [0]]))
        for j in range(links):
            p.append(np.reshape(T[j][:3, 3], (3, 1)))

        # get z vectors
        z = []
        z.append(np.array([[0], [0], [1]]))
        for k in range(links):
            z.append(np.reshape(T[k][:3, 2], (3, 1)))

        # compute jacobian
        jacob = np.zeros(shape=(6, links))

        for l in range(links):
            # computing a column at a time
            j_col = np.array(
                [[np.cross(z[l].T, (p[links] - p[l]).T, axisc=0)], [z[l]]])
            j_col = np.reshape(j_col, (1, 6))
            # print('j col',j_col)
            jacob[:, l] = j_col

        # print('Jacobian matrix: \n', jacob)

        # get current position
        current_orientation = T[6][:3, :3]
        # current_angles = GetEulerAngles2(current_orientation)

        current_pos = np.zeros((3, 1))
        current_pos[0:3, 0] = p[7].reshape(3)

        # print('desired pos: ', np.around(desired_pos.T,3))
        # print('current pos: ', np.around(current_pos.T,3))

        # current_angles = rotationMatrixToEulerAngles(current_orientation)
        pos_diff = desired_pos - current_pos
        pos_error = pos_diff[:3, 0]
        # print('pos error', pos_error)

        # compute error between the desired and current joint position
        or_error = GetOrientationError(current_orientation, R)

        error = np.zeros(6)
        error[0:3] = pos_error
        error[3:6] = or_error
        error = error.reshape((6, 1))
        # print('error: ', np.around(error.T,3))

        jacob_inv = None
        if method == "svd":
            jacob_inv = GetJacobianInverse_SVD(jacob)

        elif method == "transpose":
            jacob_inv = get_alpha(error, jacob)*jacob.T

        elif method == "pseudo":
            # jacob_inv = np.linalg.pinv(jacob)
            jacob_inv = GetJacobianInverse_pseduo(jacob)

        # compute the new joint parameters
        dq = np.matmul(jacob_inv, error)
        q = q + dq.reshape(7)  # update
        # print('new joint parameters: ',q)

        # e = np.linalg.norm(np.matmul(jacob, dq) - error)
        # print('e: ', e)
        # or (current_iteration == max_iterations):
        if (np.linalg.norm(error) <= error_threshold):
            # print('iterations: ', 'current tolerance: ', 'given tolerance',
            # current_iteration, np.linalg.norm(error), error_threshold)
            break

    # print('final q:\n', q)
    # print('\n\n')

    return np.array(q, dtype=np.float32), np.array(current_pos, dtype=np.float32), np.array(current_orientation, dtype=np.float32), np.array(error, dtype=np.float32), current_iteration, (time.time() - start_time)*1000
