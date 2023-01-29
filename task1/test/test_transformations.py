import numpy as np
import task1.transformations


def test_rot():
    theta = np.pi
    result = task1.transformations.rot(theta).round(3)
    expected = np.array([
        [-1, 0],
        [0, -1]
    ]).round(3)
    assert np.array_equal(result, expected)


def test_rotx():
    theta = np.pi
    result = task1.transformations.rotx(theta).round(3)
    expected = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ]).round(3)
    assert np.array_equal(result, expected)


def test_roty():
    theta = np.pi
    result = task1.transformations.roty(theta).round(3)
    expected = np.array([
        [-1, 0, 0],
        [0, 1, 0],
        [-0, 0, -1]
    ]).round(3)
    assert np.array_equal(result, expected)


def test_rotz():
    theta = np.pi
    result = task1.transformations.rotz(theta).round(3)
    expected = np.array([
        [-1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]
    ]).round(3)
    assert np.array_equal(result, expected)


def test_rot2trans():
    r = np.identity(3)
    result = task1.transformations.rot2trans(r).round(3)
    expected = np.identity(4).round(3)
    assert np.array_equal(result, expected)


def test_trans():
    t = np.array([2, 2])  # depending on your implementation, t = (2, 2) may also be possible
    result = task1.transformations.trans(t).round(3)
    expected = np.array([
        [1, 0, 2],
        [0, 1, 2],
        [0, 0, 1]
    ]).round(3)
    assert np.array_equal(result, expected)
