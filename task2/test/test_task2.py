import numpy as np
from task2.task2 import *


# TODO: Extend tests for more test cases if necessary.

def test_forward_kinematics():
    robot = Robot()
    result = robot.forward_kinematics(np.radians(40), np.radians(30), np.radians(-10)).round(3)
    expected = (2.525, 1.973, 0.821, 1.000)
    assert np.array_equal(result, expected)



def test_inverse_kinematics():
    # When testing, make sure that the expected solution also meets the Elbow-Up definition.
    # The following example shows a possible Elbow-Up configuration.
    expected_alpha = np.radians(20)
    expected_beta_1 = np.radians(40)
    expected_beta_2 = np.radians(-50)

    robot = Robot()
    t_ro = rot2trans(rotz(-robot.theta)).dot(trans((-robot.x_r, -robot.y_r, -robot.r)))
    p_o = robot.forward_kinematics(expected_alpha, expected_beta_1, expected_beta_2)
    p_r = t_ro.dot(p_o)

    res_alpha, res_beta_1, res_beta_2 = robot.inverse_kinematics(p_r)
    result = np.round(np.rad2deg((res_alpha, res_beta_1, res_beta_2)), 3)
    expected = np.round(np.rad2deg((expected_alpha, expected_beta_1, expected_beta_2)), 3)

    assert np.array_equal(result, expected)


def test_draw_circle():
    km_r = (1, 0, 0.55, 1)
    radius = 0.25
    positions_in_circle = 25
    robot = Robot()

    expected_alphas = np.array([0., 0.086, 0.165, 0.231, 0.281, 0.311, 0.322, 0.311,
                                0.281, 0.231, 0.165, 0.086, 0., -0.086, -0.165, -0.231,
                                -0.281, -0.311, -0.322, -0.311, -0.281, -0.231, -0.165, -0.086,
                                -0.])
    expected_beta_1s = np.array([1.036, 1.032, 1.019, 0.999, 0.971, 0.938, 0.899, 0.859, 0.818,
                                 0.78, 0.75, 0.73, 0.723, 0.73, 0.75, 0.78, 0.818, 0.859,
                                 0.899, 0.938, 0.971, 0.999, 1.019, 1.032, 1.036])
    expected_beta_2s = np.array([-0.896, -0.907, -0.938, -0.986, -1.047, -1.116, -1.186, -1.255,
                                 -1.318, -1.371, -1.412, -1.437, -1.445, -1.437, -1.412, -1.371,
                                 -1.318, -1.255, -1.186, -1.116, -1.047, -0.986, -0.938, -0.907,
                                 -0.896])

    alphas, beta_1s, beta_2s = np.round(draw_circle(robot, km_r, radius, positions_in_circle), 3)

    #print(f"{alphas}\n{beta_1s}\n{beta_2s}")

    assert np.array_equal(alphas, expected_alphas)
    assert np.array_equal(beta_1s, expected_beta_1s)
    assert np.array_equal(beta_2s, expected_beta_2s)

