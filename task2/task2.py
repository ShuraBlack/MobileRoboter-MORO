import numpy as np
from math import *
from task1.transformations import *
import matplotlib.pyplot as plt


class Robot:
    def __init__(self, x_r=2, y_r=1, theta=np.radians(30), l=0.6, h=0.2, r=0.1, a=0.1, b=0.1, l_1=0.5, l_2=0.5):
        self.x_r = x_r
        self.y_r = y_r
        self.theta = theta
        self.l = l
        self.h = h
        self.r = r
        self.a = a
        self.b = b
        self.l_1 = l_1
        self.l_2 = l_2

    def forward_kinematics(self, alpha=np.radians(0), beta_1=np.radians(0), beta_2=np.radians(0)):
        p_a2 = (0, 0, 0, 1)
        t_o_r = np.matmul(trans(np.array([self.x_r,self.y_r,self.r])), rot2trans(rotz(self.theta)))
        t_r_db = trans(np.array([(self.l / 2) - (self.a / 2), 0, self.h]))
        t_db_d = np.matmul(np.matmul(trans(np.array([0, 0, self.b])), rot2trans(rotz(alpha))), rot2trans(rotx(radians(90))))
        t_d_a1 = np.matmul(rot2trans(rotz(beta_1)), trans(np.array([self.l_1, 0, 0])))
        t_a1_a2 = np.matmul(rot2trans(rotz(beta_2)), trans(np.array([self.l_2, 0, 0])))
        return np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(t_o_r, t_r_db), t_db_d), t_d_a1), t_a1_a2),p_a2)

    def inverse_kinematics(self, p_r):
        x, y, z, _ = p_r

        alpha = atan2(y, (x - (self.l / 2 - self.a / 2)))

        t_r_db = trans(np.array([self.l / 2 - self.a / 2, 0, self.h]))

        tv_db_d = trans(np.array([0,0,self.b]))
        r_db_d_z = rot2trans(rotz(alpha))
        r_db_d_x = rot2trans(rotx(radians(90)))
        t_db_d = np.matmul(np.matmul(tv_db_d, r_db_d_z), r_db_d_x)

        t_o_d = np.matmul(t_r_db, t_db_d)
        t_d_o = np.linalg.inv(t_o_d)
        p_d = np.matmul(t_d_o, [x,y,z,1])
        x, y, z = (p_d[0], p_d[1], p_d[2])

        a = sqrt(pow(x,2) + pow(y,2))
        if not a <= self.l_1 + self.l_2:
            raise ValueError("No geometric result")

        c = (pow(a,2) - pow(self.l_2,2) - pow(self.l_2,2)) / (2 * self.l_1)
        b = -sqrt(np.absolute(pow(self.l_2,2) - pow(c,2)))

        return (alpha, atan2(y,x) - atan2(b, (self.l_1 + c)), atan2(b,c))


def draw_circle(robot, km_r=(1, 0, 0.55, 1), radius=0.25, positions_in_circle=100):
    # positions_in_circle specifies how many points of the circle to be drawn should be approached by the arm.
    # This function is not intended for plotting. Instead, it calculates the angular progression.
    alphas = []
    beta1s = []
    beta2s = []

    for t in np.linspace(0, 2 * np.pi, num=positions_in_circle, endpoint=True):
        y = km_r[1] + radius * sin(t)
        z = km_r[2] + radius * cos(t)
        a, b1, b2 = robot.inverse_kinematics((km_r[0],y,z,km_r[3]))
        alphas.append(a)
        beta1s.append(b1)
        beta2s.append(b2)

    return (alphas, beta1s, beta2s)

def draw_circle_reached_points(robot, km_r=(1, 0, 0.55, 1), radius=0.25, positions_in_circle=100):
    # positions_in_circle specifies how many points of the circle to be drawn should be approached by the arm.
    # This function is not intended for plotting. Instead, it calculates the angular progression.
    yc = []
    zc = []

    for t in np.linspace(0, 2 * np.pi, num=positions_in_circle, endpoint=True):
        y = km_r[1] + radius * sin(t)
        z = km_r[2] + radius * cos(t)

        a, b1, b2 = robot.inverse_kinematics((km_r[0],y,z,km_r[3]))
        yc.append(y)
        zc.append(z)

    return (yc,zc)


def plot_angles(alphas, beta_1s, beta_2s, positions_in_circle=100):
    plt.figure(0)
    plt.title("Winkelverläufe")
    plt.xlabel("Position im Kreis in Grad")
    plt.ylabel("Winkeleinstellung in Grad")

    for i in range(0,len(alphas)):
        alphas[i] = degrees(alphas[i])
        beta_1s[i] = degrees(beta_1s[i])
        beta_2s[i] = degrees(beta_2s[i])

    plt.xticks(range(0,360,50))
    a_label = plt.plot(alphas, '-b', label="alpha")
    b1_label = plt.plot(beta_1s, '-r', label="beta1")
    b2_label = plt.plot(beta_2s, '-g', label="beta2")

    plt.legend(loc="lower left")

    plt.savefig("gelenkwinkel.png", dpi=300)
    plt.close()
    pass


def plot_circle(reached_points):
    y,z = reached_points

    plt.figure(1)
    plt.title("Kreis (Draufsicht)")
    plt.xlabel("z Coordinates")
    plt.ylabel("y Coordinates")

    plt.scatter(z,y, c="blue")

    plt.savefig("kreis.png", dpi=300)
    plt.close()
    pass


if __name__ == '__main__':
    # Create plots with regular execution. Everything else should be run within tests.
    robot = Robot(theta=0, x_r=0, y_r=0, r=0)
    (alphas, beta_1s, beta_2s) = draw_circle(robot, positions_in_circle=360)
    plot_angles(alphas, beta_1s, beta_2s, 360)
    reached_points = draw_circle_reached_points(robot, positions_in_circle=360)
    plot_circle(reached_points)
