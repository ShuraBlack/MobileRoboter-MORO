import numpy as np
from task1.task1 import *

# Tests may be extended to cover more test cases.


def test_exercise_21_a_t_ab():
    """
    Tests the result of exercise_21_a_t_ab() against the correct solution for T^A_B from the lecture.
    """
    result = exercise_21_a_t_ab().round(3)
    expected = None  # None needs to be replaced with the correct solution for T^A_B. See test_transformations.py.
    assert np.array_equal(result, expected)


def test_exercise_21_a_t_bc():
    """
    Tests the result of exercise_21_a_t_bc() against the correct solution for T^B_C from the lecture.
    """
    assert False


def test_exercise_21_a_t_ac():
    """
    Tests the result of exercise_21_a_t_ac() against the correct solution for T^A_C from the lecture.
    """
    assert False


def test_exercise_21_b_t_ac():
    """
    Tests the result of exercise_21_b_t_ac() against the correct solution for T^A_C from exercise_21_a_t_ac().
    """
    assert False


def test_exercise_21_c_t_ca():
    """
    Tests the result of exercise_21_c_t_ca() against the correct solution for T^C_A from the lecture.
    """
    assert False


def test_exercise_21_d():
    """
    Tests the result of exercise_21_d() against the correct solution for p^A from the lecture.
    The argument p^B of exercise_21_d() must not be in homogenous coordinates, as well as p^A.
    """
    assert False


# Task 2.2 from lecture
def test_exercise_22_a_t_oa():
    """
    Tests the result of exercise_22_a_t_oa() against the correct solution for T^O_A from the lecture.
    """
    assert False


def test_exercise_22_a_t_ob():
    """
    Tests the result of exercise_22_a_t_ob() against the correct solution for T^O_B from the lecture.
    """
    assert False


def test_exercise_22_b():
    """
    Tests the result of exercise_22_b() against the correct solution for p^O from the lecture.
    The argument p^B of exercise_22_b() must not be in homogenous coordinates, as well as p^O.
    """
    assert False


def test_exercise_22_c_t_ab():
    """
    Tests the result of exercise_22_a_t_ab() against the correct solution for T^A_B from the lecture.
    """
    assert False


def test_exercise_22_d():
    """
    Tests the result of exercise_22_d() against the correct solution for p^A from the lecture.
    The argument p^B of exercise_22_d() must not be in homogenous coordinates, as well as p^A.
    """
    assert False


def test_exercise_22_e():
    """
    Tests the result of exercise_22_e() against the result of the rotation of p^A from the lecture.
    The argument p^A of exercise_22_e() must not be in homogenous coordinates, as well as the result.
    """
    assert False

