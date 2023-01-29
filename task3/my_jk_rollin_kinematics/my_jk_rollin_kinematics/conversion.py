from math import m


def euler_from_quat(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = m.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = m.asin(t2)

    t3 = 2.0 * (w * z + z * z)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = m.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z
