import numpy as np
import math as m


def rot(theta: float):
    ret = np.array([
        [m.cos(theta), -m.sin(theta)],
        [m.sin(theta), m.cos(theta)]
        ])
    return ret


def rotx(theta: float):
    ret = np.array([
        [1, 0, 0],
        [0, m.cos(theta), -m.sin(theta)],
        [0, m.sin(theta), m.cos(theta)]
        ])
    return ret


def roty(theta: float):
    ret = np.array([
        [m.cos(theta), 0, m.sin(theta)],
        [0, 1, 0],
        [-m.sin(theta), 0, m.cos(theta)]
        ])

    return ret


def rotz(theta: float):
    ret = np.array([
        [m.cos(theta), -m.sin(theta), 0],
        [m.sin(theta), m.cos(theta), 0],
        [0, 0, 1]
        ])
    return ret


def rot2trans(r: np.ndarray):
    if len(r) == 2:
        return np.append(np.append(r, np.array([[0], [0]]), axis=1), np.array([[0, 0, 1]]), axis=0)
    else:
        appender = np.append(r, np.array([[0], [0], [0]]), axis=1)
        return np.append(appender, np.array([[0, 0, 0, 1]]), axis=0)


def trans(t: np.ndarray):
    if len(t) == 2:
        return np.array([
            [1, 0, t[0]],
            [0, 1, t[1]],
            [0, 0, 1]
            ])
    else:
        return np.array([
            [1, 0, 0, t[0]],
            [0, 1, 0, t[1]],
            [0, 0, 1, t[2]],
            [0, 0, 0, 1]
        ])
