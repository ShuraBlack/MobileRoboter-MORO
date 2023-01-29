import numpy as np


class Robot:

    def __init__(
        self,
        x_r=2,
        y_r=1,
        theta=np.radians(30),
        length=0.0,
        h=0.021,
        r=0.026,
        a=0.12,
        b=0.047,
        l_1=0.093,
        l_2=0.082,
            l_3=0.051):

        self.x_r = x_r
        self.y_r = y_r
        self.theta = theta
        self.length = length
        self.h = h
        self.r = r
        self.a = a
        self.b = b
        self.l_1 = l_1
        self.l_2 = l_2
        self.l_3 = l_3
