#!../venvs/robot_env/bin/python3.7
from __future__ import print_function
# import rosnode
# import tf_conversions
# import threading
# import roslib; roslib.load_manifest('teleop_twist_keyboard')
# import rospy
# from geometry_msgs.msg import Twist
# import random
# import sys, select, termios, tty
import math
import numpy as np
from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp, options
from cvxopt import matrix, sparse
# import itertools
from scipy.special import comb

import math
import time
import numpy as np
from cvxopt import matrix
from cvxopt.solvers import qp, options
from cvxopt import matrix, sparse
# from geometry_msgs.msg import TransformStamped, PoseStamped
from scipy.special import comb
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#from utilities.Telemetry import Telemetry
N=4
x_rand_span_x = 0.085 * np.random.randint(1, 2, (1, N))  # setting up position error range for each robot,
x_rand_span_y = 0.085 * np.random.randint(1, 2, (1, N))
XRandSpan = np.concatenate((x_rand_span_x, x_rand_span_y))
v_rand_span = 0.005 * np.ones((2, N))  # setting up velocity error range for each robot


def create_clf_unicycle_pose_controller(approach_angle_gain=1, desired_angle_gain=2.7, rotation_error_gain=1):
    """Returns a controller ($u: \mathbf{R}^{3 \times N} \times \mathbf{R}^{3 \times N} \to \mathbf{R}^{2 \times N}$)
    that will drive a unicycle-modeled agent to a pose (i.e., position & orientation). This control is based on a control
    Lyapunov function.

    approach_angle_gain - affects how the unicycle approaches the desired position
    desired_angle_gain - affects how the unicycle approaches the desired angle
    rotation_error_gain - affects how quickly the unicycle corrects rotation errors.


    -> function
    """

    gamma = approach_angle_gain
    k = desired_angle_gain
    h = rotation_error_gain

    def R(theta):
        return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    def pose_uni_clf_controller(states, poses):
        N_states = states.shape[1]
        dxu = np.zeros((2, N_states))

        for i in range(N_states):
            # print(states.shape)
            # print(poses.shape)
            translate = R(-poses[2, i]).dot((poses[:2, i] - states[:2, i]))
            e = np.linalg.norm(translate)
            theta = np.arctan2(translate[1], translate[0])
            alpha = theta - (states[2, i] - poses[2, i])
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

            ca = np.cos(alpha)
            sa = np.sin(alpha)

            # print(gamma)
            # print(e)
            # print(ca)

            dxu[0, i] = gamma * e * ca
            dxu[1, i] = k * alpha + gamma * ((ca * sa) / alpha) * (alpha + h * theta)

        return dxu

    def pose_uni_clf_controller_custom(states, poses):
        # CUSTOM CONTROLLER USING GOALS WITHOUT SPECIFIC THETA VALUES
        # TODO: figure out meaning of gamma value in above function

        # states = current
        # poses = goal
        N_states = states.shape[1]
        print(N_states)
        dxu = np.zeros((2, N_states))

        for i in range(N_states):
            theta = states[2, i]

            # calculate distance left, alpha, and Oc
            d = math.sqrt(((poses[0,i] - states[0, i]) ** 2) + ((poses[1,i] - states[1, i]) ** 2))
            alpha = math.atan2(poses[1,i] - states[1, i], poses[0,i] - states[0, i])
            Oc = alpha - theta
            Oc = np.arctan2(np.sin(Oc), np.cos(Oc))

            if d >= 0.05:
                w = (np.pi / 2) * math.sin(Oc)

                # TODO: check velocity math (goes with understanding gamma in above function)
                v = d
                if v > 0.08:
                    v = 0.08

            else:
                # set robot to stop
                v = 0
                w = 0

            dxu[0, i] = v
            dxu[1, i] = w

        return dxu

    return pose_uni_clf_controller_custom

def create_si_to_uni_mapping(projection_distance=0.05, angular_velocity_limit=np.pi):
    """Creates two functions for mapping from single integrator dynamics to
    unicycle dynamics and unicycle states to single integrator states.

    This mapping is done by placing a virtual control "point" in front of
    the unicycle.

    projection_distance: How far ahead to place the point
    angular_velocity_limit: The maximum angular velocity that can be provided

    -> (function, function)
    """

    # Check user input types
    assert isinstance(projection_distance, (int,
                                            float)), "In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type %r." % type(
        projection_distance).__name__
    assert isinstance(angular_velocity_limit, (int,
                                               float)), "In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be an integer or float. Recieved type %r." % type(
        angular_velocity_limit).__name__

    # Check user input ranges/sizes
    assert projection_distance > 0, "In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be positive. Recieved %r." % projection_distance
    assert projection_distance >= 0, "In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be greater than or equal to zero. Recieved %r." % angular_velocity_limit

    def si_to_uni_dyn(dxi, poses):
        """Takes single-integrator velocities and transforms them to unicycle
        control inputs.

        dxi: 2xN numpy array of single-integrator control inputs
        poses: 3xN numpy array of unicycle poses

        -> 2xN numpy array of unicycle control inputs
        """

        # Check user input types
        assert isinstance(dxi,
                          np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type %r." % type(
            dxi).__name__
        assert isinstance(poses,
                          np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(
            poses).__name__

        # Check user input ranges/sizes
        assert dxi.shape[
                   0] == 2, "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % \
                            dxi.shape[0]
        assert poses.shape[
                   0] == 3, "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % \
                            poses.shape[0]
        assert dxi.shape[1] == poses.shape[
            1], "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size %r x %r and current pose array of size %r x %r." % (
        dxi.shape[0], dxi.shape[1], poses.shape[0], poses.shape[1])

        M, N = np.shape(dxi)

        cs = np.cos(poses[2, :])
        ss = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = (cs * dxi[0, :] + ss * dxi[1, :])
        dxu[1, :] = (1 / projection_distance) * (-ss * dxi[0, :] + cs * dxi[1, :])

        # Impose angular velocity cap.
        dxu[1, dxu[1, :] > angular_velocity_limit] = angular_velocity_limit
        dxu[1, dxu[1, :] < -angular_velocity_limit] = -angular_velocity_limit

        return dxu

    def uni_to_si_states(poses):
        """Takes unicycle states and returns single-integrator states

        poses: 3xN numpy array of unicycle states

        -> 2xN numpy array of single-integrator states
        """

        _, N = np.shape(poses)

        si_states = np.zeros((2, N))
        si_states[0, :] = poses[0, :] + projection_distance * np.cos(poses[2, :])
        si_states[1, :] = poses[1, :] + projection_distance * np.sin(poses[2, :])

        return si_states

    return si_to_uni_dyn, uni_to_si_states


def create_uni_to_si_dynamics(projection_distance=0.05):
    """Creates two functions for mapping from unicycle dynamics to single
    integrator dynamics and single integrator states to unicycle states.

    This mapping is done by placing a virtual control "point" in front of
    the unicycle.

    projection_distance: How far ahead to place the point

    -> function
    """

    # Check user input types
    assert isinstance(projection_distance, (int,
                                            float)), "In the function create_uni_to_si_dynamics, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type %r." % type(
        projection_distance).__name__

    # Check user input ranges/sizes
    assert projection_distance > 0, "In the function create_uni_to_si_dynamics, the projection distance of the new control point (projection_distance) must be positive. Recieved %r." % projection_distance

    def uni_to_si_dyn(dxu, poses):
        """A function for converting from unicycle to single-integrator dynamics.
        Utilizes a virtual point placed in front of the unicycle.

        dxu: 2xN numpy array of unicycle control inputs
        poses: 3xN numpy array of unicycle poses
        projection_distance: How far ahead of the unicycle model to place the point

        -> 2xN numpy array of single-integrator control inputs
        """

        # Check user input types
        assert isinstance(dxu,
                          np.ndarray), "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the unicycle velocity inputs (dxu) must be a numpy array. Recieved type %r." % type(
            dxi).__name__
        assert isinstance(poses,
                          np.ndarray), "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(
            poses).__name__

        # Check user input ranges/sizes
        assert dxu.shape[
                   0] == 2, "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the dimension of the unicycle velocity inputs (dxu) must be 2 ([v;w]). Recieved dimension %r." % \
                            dxu.shape[0]
        assert poses.shape[
                   0] == 3, "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % \
                            poses.shape[0]
        assert dxu.shape[1] == poses.shape[
            1], "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the number of unicycle velocity inputs must be equal to the number of current robot poses. Recieved a unicycle velocity input array of size %r x %r and current pose array of size %r x %r." % (
        dxu.shape[0], dxu.shape[1], poses.shape[0], poses.shape[1])

        M, N = np.shape(dxu)

        cs = np.cos(poses[2, :])
        ss = np.sin(poses[2, :])

        dxi = np.zeros((2, N))
        dxi[0, :] = (cs * dxu[0, :] - projection_distance * ss * dxu[1, :])
        dxi[1, :] = (ss * dxu[0, :] + projection_distance * cs * dxu[1, :])

        return dxi

    return uni_to_si_dyn

def trap_cdf_inv(a, c, delta, sigma):
    # returns list of b2, b1, sigma
    b2 = delta
    b1 = delta

    # a and c should be positive

    if a > c:  # [-A, A] is the large one, and[-C, C] is the smaller one
        A = a
        C = c
    else:
        A = c
        C = a

    if A == 0 and C == 0:
        return b2, b1, sigma

    # O_vec = [-(A + C), -(A - C), (A - C), (A + C)] # vector of vertices on the trap distribution cdf

    h = 1 / (2 * A)  # height of the trap distribution
    area_seq = [1 / 2 * 2 * C * h, 2 * (A - C) * h, 1 / 2 * 2 * C * h]
    area_vec = [area_seq[0], sum(area_seq[:2])]

    if abs(A - C) < 1e-5:  # then is triangle
        # assuming sigma > 50
        b1 = (A + C) - 2 * C * np.sqrt((1 - sigma) / (1 - area_vec[1]))  # 1 - area_vec[1] should be very close to 0.5
        b2 = -b1

        b1 = b1 + delta
        b2 = b2 + delta  # apply shift here due to xi - xj

    else:  # than is trap
        if sigma > area_vec[1]:  # right triangle area
            b1 = (A + C) - 2 * C * np.sqrt((1 - sigma) / (1 - area_vec[1]))
            b2 = -(A + C) + 2 * C * np.sqrt((1 - sigma) / (1 - area_vec[1]))

            b1 = b1 + delta
            b2 = b2 + delta  # apply shift here due to xi - xj

        elif sigma > area_vec[0] and sigma <= area_vec[1]:  # in between the triangle part
            b1 = -(A - C) + (sigma - area_vec[0]) / h  # assuming > 50%, then b1 should > 0
            b2 = -b1

            b1 = b1 + delta
            b2 = b2 + delta  # apply shift here due to xi - xj

            # note that b1 could be > or < b2, depending on whether sigma > or < .5

        elif sigma <= area_vec[0]:
            b1 = -(A + C) + 2 * C * np.sqrt(sigma / area_vec[0])  # assuming > 50%, then b1 should > 0
            b2 = -b1

            b1 = b1 + delta
            b2 = b2 + delta  # apply shift here due to xi - xj

        else:
            print('first triangle, which is not allowed as long as we assume sigma > 50%')

    return b2, b1, sigma

def create_si_pr_barrier_certificate_centralized(gamma=100, safety_radius=0.2, magnitude_limit=0.2, confidence_level=1.0,
                                                 XRandSpan=XRandSpan, URandSpan=v_rand_span):

    def barrier_certificate(dxi, x, XRandSpan, URandSpan):
        # print(XRandSpan)
        # N = dxi.shape[1]
        num_constraints = int(comb(N, 2))
        A = np.zeros((num_constraints, 2 * N))
        b = np.zeros(num_constraints)
        H = sparse(matrix(2 * np.identity(2 * N)))

        count = 0
        if len(XRandSpan) == 1:
            XRandSpan = np.zeros(2, N)
        if len(URandSpan) == 1:
            URandSpan = np.zeros(2, N)
        for i in range(N - 1):
            for j in range(i + 1, N):

                max_dvij_x = np.linalg.norm(URandSpan[0, i] + URandSpan[0, j])
                max_dvij_y = np.linalg.norm(URandSpan[1, i] + URandSpan[1, j])
                max_dxij_x = np.linalg.norm(x[0, i] - x[0, j]) + np.linalg.norm(XRandSpan[0, i] + XRandSpan[0, j])
                max_dxij_y = np.linalg.norm(x[1, i] - x[1, j]) + np.linalg.norm(XRandSpan[1, i] + XRandSpan[1, j])
                BB_x = -safety_radius ** 2 - 2 / gamma * max_dvij_x * max_dxij_x
                BB_y = -safety_radius ** 2 - 2 / gamma * max_dvij_y * max_dxij_y
                b2_x, b1_x, sigma = trap_cdf_inv(XRandSpan[0, i], XRandSpan[0, j], x[0, i] - x[0, j], confidence_level)
                b2_y, b1_y, sigma = trap_cdf_inv(XRandSpan[1, i], XRandSpan[1, j], x[1, i] - x[1, j], confidence_level)

                if (b2_x < 0 and b1_x > 0) or (b2_x > 0 and b1_x < 0):
                    # print('WARNING: distance between robots on x smaller than error bound!')
                    b_x = 0
                elif (b1_x < 0) and (b2_x < b1_x) or (b2_x < 0 and b2_x > b1_x):
                    b_x = b1_x
                elif (b2_x > 0 and b2_x < b1_x) or (b1_x > 0 and b2_x > b1_x):
                    b_x = b2_x
                else:
                    b_x = b1_x
                    # print('WARNING: no uncertainty or sigma = 0.5 on x')  # b1 = b2 or no uncertainty

                if (b2_y < 0 and b1_y > 0) or (b2_y > 0 and b1_y < 0):
                    # print('WARNING: distance between robots on y smaller than error bound!')
                    b_y = 0
                elif (b1_y < 0 and b2_y < b1_y) or (b2_y < 0 and b2_y > b1_y):
                    b_y = b1_y
                elif (b2_y > 0 and b2_y < b1_y) or (b1_y > 0 and b2_y > b1_y):
                    b_y = b2_y
                else:
                    b_y = b1_y

                A[count, (2 * i)] = -2 * b_x  # matlab original: A(count, (2*i-1):(2*i)) = -2*([b_x;b_y]);
                A[count, (2 * i + 1)] = -2 * b_y

                A[count, (2 * j)] = 2 * b_x  # matlab original: A(count, (2*j-1):(2*j)) =  2*([b_x;b_y])';
                A[count, (2 * j + 1)] = 2 * b_y

                t1 = np.array([[b_x], [0.0]])
                t2 = np.array([[max_dvij_x], [0]])
                t3 = np.array([[max_dxij_x], [0]])
                t4 = np.array([[0], [b_y]])
                t5 = np.array([[0], [max_dvij_y]])
                t6 = np.array([[0], [max_dxij_y]])


                h1 = np.linalg.norm(t1) ** 2 - safety_radius ** 2 - 2 * np.linalg.norm(
                    t2) * np.linalg.norm(t3) / gamma
                h2 = np.linalg.norm(t4) ** 2 - safety_radius ** 2 - 2 * np.linalg.norm(
                    t5) * np.linalg.norm(t6) / gamma  # h_y

                h = h1 + h2

                b[count] = gamma * h ** 3  # matlab original: b(count) = gamma*h^3
                count += 1

        # Threshold control inputs before QP
        norms = np.linalg.norm(dxi, 2, 0)
        idxs_to_normalize = (norms > magnitude_limit)
        dxi[:, idxs_to_normalize] = dxi[:, idxs_to_normalize] * (magnitude_limit / norms[idxs_to_normalize])

        f_mat = -2 * np.reshape(dxi, 2 * N, order='F')
        f_mat = f_mat.astype('float')
        result = qp(H, matrix(f_mat), matrix(A), matrix(b))['x']

        return np.reshape(result, (2, -1), order='F')

    return barrier_certificate


def create_pr_unicycle_barrier_certificate_cent(barrier_gain=100, safety_radius=0.12, projection_distance=0.05,
                                                magnitude_limit=0.2, confidence_level=1.0, XRandSpan=XRandSpan, URandSpan=v_rand_span):

    # Check user input types
    assert isinstance(barrier_gain, (int,
                                     float)), "In the function create_pr_unicycle_barrier_certificate, the barrier gain (barrier_gain) must be an integer or float. Recieved type %r." % type(
        barrier_gain).__name__
    assert isinstance(safety_radius, (int,
                                      float)), "In the function create_pr_unicycle_barrier_certificate, the safe distance between robots (safety_radius) must be an integer or float. Recieved type %r." % type(
        safety_radius).__name__
    assert isinstance(projection_distance, (int,
                                            float)), "In the function create_pr_unicycle_barrier_certificate, the projected point distance for the diffeomorphism between sinlge integrator and unicycle (projection_distance) must be an integer or float. Recieved type %r." % type(
        projection_distance).__name__
    assert isinstance(magnitude_limit, (int,
                                        float)), "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be an integer or float. Recieved type %r." % type(
        magnitude_limit).__name__
    assert isinstance(confidence_level,
                      float), "In the function create_pr_unicycle_barrier_certificate, the confidence level must be a float. Recieved type %r." % type(
        confidence_level).__name__

    # Check user input ranges/sizes

    assert barrier_gain > 0, "In the function create_pr_unicycle_barrier_certificate, the barrier gain (barrier_gain) must be positive. Recieved %r." % barrier_gain
    assert safety_radius >= 0.12, "In the function create_pr_unicycle_barrier_certificate, the safe distance between robots (safety_radius) must be greater than or equal to the diameter of the robot (0.12m). Recieved %r." % safety_radius
    assert projection_distance > 0, "In the function create_pr_unicycle_barrier_certificate, the projected point distance for the diffeomorphism between sinlge integrator and unicycle (projection_distance) must be positive. Recieved %r." % projection_distance
    assert magnitude_limit > 0, "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be positive. Recieved %r." % magnitude_limit
    assert magnitude_limit <= 0.2, "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit
    assert confidence_level <= 1, "In the function create_pr_unicycle_barrier_certificate, the confidence level must be less than 1. Recieved %r." % confidence_level
    assert confidence_level >= 0, "In the function create_pr_unicycle_barrier_certificate, the confidence level must be positive (greater than 0). Recieved %r." % confidence_level

    si_barrier_cert = create_si_pr_barrier_certificate_centralized(gamma=barrier_gain,
                                                                   safety_radius=safety_radius + projection_distance,
                                                                   confidence_level=confidence_level, XRandSpan=XRandSpan,
                                                                   URandSpan=v_rand_span)

    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping(projection_distance=projection_distance)

    uni_to_si_dyn = create_uni_to_si_dynamics(projection_distance=projection_distance)

    def f(dxu, x, XRandSpan=XRandSpan, URandSpan=v_rand_span):



        # Check user input types
        assert isinstance(dxu,
                          np.ndarray), "In the function created by the create_unicycle_barrier_certificate function, the unicycle robot velocity command (dxu) must be a numpy array. Recieved type %r." % type(
            dxu).__name__
        assert isinstance(x,
                          np.ndarray), "In the function created by the create_unicycle_barrier_certificate function, the robot states (x) must be a numpy array. Recieved type %r." % type(
            x).__name__

        # Check user input ranges/sizes
        assert x.shape[
                   0] == 3, "In the function created by the create_unicycle_barrier_certificate function, the dimension of the unicycle robot states (x) must be 3 ([x;y;theta]). Recieved dimension %r." % \
                            x.shape[0]
        assert dxu.shape[
                   0] == 2, "In the function created by the create_unicycle_barrier_certificate function, the dimension of the robot unicycle velocity command (dxu) must be 2 ([v;w]). Recieved dimension %r." % \
                            dxu.shape[0]
        assert x.shape[1] == dxu.shape[
            1], "In the function created by the create_unicycle_barrier_certificate function, the number of robot states (x) must be equal to the number of robot unicycle velocity commands (dxu). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (
        x.shape[0], x.shape[1], dxu.shape[0], dxu.shape[1])

        x_si = uni_to_si_states(x)
        # Convert unicycle control command to single integrator one
        dxi = uni_to_si_dyn(dxu, x)
        # Apply single integrator barrier certificate
        dxi = si_barrier_cert(dxi, x_si, XRandSpan, URandSpan)
        # Return safe unicycle command
        return si_to_uni_dyn(dxi, x)

    return f



# rospy.init_node('teleop_twist_keyboard')
# publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
# rospy.sleep(2)
# twist = Twist()
# uni_controller = create_clf_unicycle_pose_controller()
# uni_barrier_cert = create_pr_unicycle_barrier_certificate_cent(barrier_gain=barrier_gain, projection_distance=projection_distance, magnitude_limit=magnitude_limit, safety_radius=safety_radius, confidence_level=confidence_level, XRandSpan=XRandSpan, URandSpan=v_rand_span)





# def callback(data, args):

# 	i = args

# 	theta = tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]
# 	x[0,i] = data.pose.position.x
# 	x[1,i] = data.pose.position.y
# 	x[2,i] = theta

# def control_callback(event):
#     p = 0
#     dxu = uni_controller(x, goal_points)
#     dxu = uni_barrier_cert(dxu, x)

#     twist.linear.x = dxu[0,p]/50.
#     twist.linear.y = 0.0
#     twist.linear.z = 0.0
#     twist.angular.x = 0
#     twist.angular.y = 0
#     twist.angular.z = dxu[1,p]/25.
#     publisher.publish(twist)

# def central():

	
	# rospy.Subscriber('/vrpn_client_node/Hus117'  + '/pose', PoseStamped, callback, 0 ) 
	# rospy.Subscriber('/vrpn_client_node/Hus137'  + '/pose', PoseStamped, callback, 1 ) 
	# rospy.Subscriber('/vrpn_client_node/Hus138'  + '/pose', PoseStamped, callback, 2 ) 
	# rospy.Subscriber('/vrpn_client_node/Hus188'  + '/pose', PoseStamped, callback, 3 ) 

	
	# timer = rospy.Timer(rospy.Duration(0.05), control_callback)
	# rospy.spin()

# first_time = 0
def init():
    # global first_time
    # first_time = time.time()
    ax.set_xlim(-3.0, 3.0)
    ax.set_ylim(-3.0, 3.0)
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ln.set_data(x[0,:], x[1,:])
    return ln,

def go_to_goal_controller(curr, goal):
    K = [1, 1]
    u = [0.0, 0.0]
    v_max = 1

    goal_x, goal_y = goal
    curr_x, curr_y = curr
    err_x = goal_x - curr_x
    err_y = goal_y - curr_y

    if np.linalg.norm(np.array([err_x, err_y])) < 0.25:
        return np.array([0, 0])
    
    u[0] = K[0] * err_x
    u[1] = K[1] * err_y

    vx = u[0]
    vy = u[1]

    if abs(vx) > v_max:
        vx = np.sign(vx) * v_max
    if abs(vy) > v_max:
        vy = np.sign(vy) * v_max

    return np.array([vx, vy])


def update(frame):
    # global first_time
    global x
    # next_time = first_time
    # first_time = time.time()
    # dt = first_time - next_time
    # print("before!", x)
    # p = 0
    # dxu = uni_controller(x, goal_points)
    # print(dxu)
    # dxu = uni_barrier_cert(dxu, x)
    
    for i in range(4):
        dxi = go_to_goal_controller(x[:2,i], goal_points[:2, i])
        print(dxi)
        vels = np.zeros((2,4))
        vels[:, i] = dxi
        dx = barrier_cert(vels, x, XRandSpan, v_rand_span)
        print(dx[:, i])
        # vx = dxi[0]
        # vy = dxi[1]
        vx = dx[0, i]
        vy = dx[1, i]
        x[0, i] += vx * 0.01
        x[1, i] += vy * 0.01


        # theta = x[2,i]
        # vx = dxu[0,i]      # /5.0
        # omega = dxu[1, i] #/ 2.5
        # theta += omega * 0.01
        # print('vx={}, omega={}'.format(vx, omega))
        # x[0,i] += vx * np.cos(theta) * 0.01
        # x[1,i] += vx * np.sin(theta) * 0.01
        # theta = np.arctan2(np.sin(theta))
        # x[2,i] = theta
        # print(x)

    xdata=x[0,:]
    ydata=x[1,:]
    # print(xdata)
    # print(ydata)
    ln.set_data(xdata, ydata)
    return ln,


if __name__ == '__main__':
    # Disable output of CVXOPT
    options['show_progress'] = False
    # Change default options of CVXOPT for faster solving
    options['reltol'] = 1e-2 # was e-2
    options['feastol'] = 1e-2 # was e-4
    options['maxiters'] = 50 # default is 100

    N = 4
    goal_points = np.array([[0., 0., 2., -2.], [-2., 2., 0., 0.], [math.pi / 2, -math.pi / 2, math.pi, 0.]])
    v_rand_span = 0.005 * np.ones((2, N))  # setting up velocity error range for each robot

    x_rand_span_x = 0 * np.random.randint(1, 2, (1, N))  # setting up position error range for each robot,
    x_rand_span_y = 0 * np.random.randint(1, 2, (1, N))
    XRandSpan = np.concatenate((x_rand_span_x, x_rand_span_y))
    Kp = 10
    Vmax = 100
    Wmax = np.pi
    safety_radius = 0.4
    barrier_gain = 10000
    projection_distance = 0.05
    magnitude_limit = 0.2
    confidence_level = 0.8 #1.0
    dxu = np.array([[0,0,0,0],[0,0,0,0]])
    dxi = np.array([[0,0,0,0],[0,0,0,0]])
    x = np.array([[0,0,-2,2],[2,-2,0,0],[0,0,0,math.pi]], dtype=np.float16)
 
    barrier_cert = create_si_pr_barrier_certificate_centralized(safety_radius=0.4, magnitude_limit=1, confidence_level=0.5)
    fig, ax = plt.subplots()
    xdata, ydata = x[0,:],x[1,:]
    ln, = ax.plot(xdata, ydata, 'ro', markersize=5)
    ani = FuncAnimation(fig, update, frames=range(10000000), init_func=init, blit=True,interval=10)
    plt.show()


    # central()
	# try:
	# 	central()
	# except rospy.ROSInterruptException:
	# 	print(rospy.ROSInterruptException)