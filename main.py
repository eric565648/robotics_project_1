import numpy as np
from math import sin, cos, tan, atan2, acos, asin, degrees, radians, pi
import copy

'''
while True:
    mode = input("Calculate kinematics or inverse kinematics? (k or ik) ")
    if mode == 'k' or mode == 'ik':
        break
'''
# mode = 'k'
# mode_flag = mode

a1 = 0.12
a2 = 0.25
a3 = 0.26

def show_angle(ans):

    # Show the result of ik

    print ""
    i = 0
    for thetas in ans:
        t1 = degrees(thetas['theta_1'])
        t2 = degrees(thetas['theta_2'])
        t3 = degrees(thetas['theta_3'])
        t4 = degrees(thetas['theta_4'])
        t5 = degrees(thetas['theta_5'])
        t6 = degrees(thetas['theta_6'])

        print "Ans ", i
        print "  ", format(t1, '.4f'), "  ", format(t2, '.4f'), "  ", format(t3, '.4f'), "  ", format(t4, '.4f'), "  ", format(t5, '.4f'), "  ", format(t6, '.4f'),
        print ""
        print "---------------------------"
        i+=1

    print "TAs are the best. TAT"
    print ""

def inverse_kinematics(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz):

    ans = []
    angle = ['theta_1', 'theta_2', 'theta_3', 'theta_4', 'theta_5', 'theta_6']
    set = dict.fromkeys(angle)

    ans.append(copy.deepcopy(set))

    # Theta 1
    # theta 1 = atan2(py/px)
    ans[0]['theta_1'] = atan2(py, px)
    ans.append(copy.deepcopy(ans[0]))
    ans[1]['theta_1'] = atan2(-1*py, -1*px)

    # Theta 3
    # theta 3 = acos((px**2 + py**2 + pz**2 - 2*a1*(cos(t1)*px + sin(t1)*py) - a2**2 - a3**2 + a1**2) / (2*a2*a3))
    ans_dummy = []
    delete = []
    for thetas in ans:
        t1 = thetas['theta_1']

        t3 = ((px**2 + py**2 + pz**2 - 2*a1*(cos(t1)*px + sin(t1)*py) - a2**2 - a3**2 + a1**2) / (2*a2*a3))
        if -1 <= t3 <= 1:
            t3 = acos((px**2 + py**2 + pz**2 - 2*a1*(cos(t1)*px + sin(t1)*py) - a2**2 - a3**2 + a1**2) / (2*a2*a3))
            thetas['theta_3'] = t3
            ans_dummy.append(copy.deepcopy(thetas))
            ans_dummy[-1]['theta_3'] = -1*t3
        else:
            delete.append(ans.index(thetas))

    ans.extend(ans_dummy)
    for i in delete:
        del(ans[i])

    # Theta 2
    # theta 2 = atan2(1*a2*sin(t3), (a2*cos(t3) + a3)) - t3 - phi
    # where phi = atan2(pz, cos(t1)*px + sin(t1)*py - a1)
    ans_dummy = []
    delete = []
    for thetas in ans:
        t1 = thetas['theta_1']
        t3 = thetas['theta_3']

        phi = atan2(pz, cos(t1)*px + sin(t1)*py - a1)
        t2 = atan2(1*a2*sin(t3), (a2*cos(t3) + a3)) - t3 - phi

        thetas['theta_2'] = t2


    # Theta 4
    # theta 4 = atan2(-1*sin(t2+t3)*(cos(t1)*ax + sin(t1)*ay) - cos(t2 + t3)*az, cos(t2+t3)*(cos(t1)*ax + sin(t1)*ay) - sin(t2 + t3)*az)
    ans_dummy = []
    delete = []
    for thetas in ans:
        t1 = thetas['theta_1']
        t2 = thetas['theta_2']
        t3 = thetas['theta_3']

        t4 = atan2(-1*sin(t2+t3)*(cos(t1)*ax + sin(t1)*ay) - cos(t2 + t3)*az, cos(t2+t3)*(cos(t1)*ax + sin(t1)*ay) - sin(t2 + t3)*az)
        thetas['theta_4'] = t4
        ans_dummy.append(copy.deepcopy(thetas))
        ans_dummy[-1]['theta_4'] = atan2(-1*(-1*sin(t2+t3)*(cos(t1)*ax + sin(t1)*ay) - cos(t2 + t3)*az), -1*(cos(t2+t3)*(cos(t1)*ax + sin(t1)*ay) - sin(t2 + t3)*az))

    ans.extend(ans_dummy)

    # Theta 5
    # theta 5 = atan2(cos(t2+t3+t4)*(cos(t1)*ax + sin(t1)*ay) - sin(t2+t3+t4)*az, cos(t1)*ay - sin(t1)*ax)
    ans_dummy = []
    delete = []
    for thetas in ans:
        t1 = thetas['theta_1']
        t2 = thetas['theta_2']
        t3 = thetas['theta_3']
        t4 = thetas['theta_4']

        t5 = atan2(cos(t2+t3+t4)*(cos(t1)*ax + sin(t1)*ay) - sin(t2+t3+t4)*az, cos(t1)*ay - sin(t1)*ax)
        thetas['theta_5'] = t5

    # Theta 6
    # theta 6 = atan2(cos(t2+t3+t4)*(cos(t1)*ax + sin(t1)*ay) - sin(t2+t3+t4)*az, cos(t1)*ay - sin(t1)*ax)
    ans_dummy = []
    delete = []
    for thetas in ans:
        t1 = thetas['theta_1']
        t2 = thetas['theta_2']
        t3 = thetas['theta_3']
        t4 = thetas['theta_4']
        t5 = thetas['theta_5']

        t6 = atan2(-1*sin(t2+t3+t4)*(cos(t1)*nx + sin(t1)*ny) - cos(t2 + t3 + t4)*nz, -1*sin(t2+t3+t4)*(cos(t1)*ox + sin(t1)*oy) - cos(t2 + t3 + t4)*oz)
        thetas['theta_6'] = t6

    show_angle(ans)

    return 0

def kinematics(t1, t2, t3, t4, t5, t6):

    # Initialize the matrixes

    A1 = np.zeros((4, 4), dtype=float)
    A1[0, 0] = cos(t1)
    A1[0, 2] = -sin(t1)
    A1[0, 3] = a1*cos(t1)
    A1[1, 0] = sin(t1)
    A1[1, 2] = cos(t1)
    A1[1, 3] = a1*sin(t1)
    A1[2, 1] = -1
    A1[3, 3] = 1

    A2 = np.zeros((4, 4), dtype=float)
    A2[0, 0] = cos(t2)
    A2[0, 1] = -sin(t2)
    A2[0, 3] = a2*cos(t2)
    A2[1, 0] = sin(t2)
    A2[1, 1] = cos(t2)
    A2[1, 3] = a2*sin(t2)
    A2[2, 2] = 1
    A2[3, 3] = 1

    # Multiply A1 A2
    T12 = np.matmul(A1, A2)

    A3 = np.zeros((4, 4), dtype=float)
    A3[0, 0] = cos(t3)
    A3[0, 1] = -sin(t3)
    A3[0, 3] = a3*cos(t3)
    A3[1, 0] = sin(t3)
    A3[1, 1] = cos(t3)
    A3[1, 3] = a3*sin(t3)
    A3[2, 2] = 1
    A3[3, 3] = 1

    # Multiply A1 A2 A3
    T13 = np.matmul(T12, A3)

    A4 = np.zeros((4, 4), dtype=float)
    A4[0, 0] = cos(t4)
    A4[0, 2] = -sin(t4)
    A4[1, 0] = sin(t4)
    A4[1, 2] = cos(t4)
    A4[2, 1] = -1
    A4[3, 3] = 1

    # Multiply A1 A2 A3 A4
    T14 = np.matmul(T13, A4)

    A5 = np.zeros((4, 4), dtype=float)
    A5[0, 0] = cos(t5)
    A5[0, 2] = sin(t5)
    A5[1, 0] = sin(t5)
    A5[1, 2] = -cos(t5)
    A5[2, 1] = 1
    A5[3, 3] = 1

    # Multiply A1 A2 A3 A4 A5
    T15 = np.matmul(T14, A5)

    A6 = np.zeros((4, 4), dtype=float)
    A6[0, 0] = cos(t6)
    A6[0, 1] = -sin(t6)
    A6[1, 0] = sin(t6)
    A6[1, 1] = cos(t6)
    A6[2, 2] = 1
    A6[3, 3] = 1

    # Multiply A1 A2 A3 A4 A5 A6
    T16 = np.matmul(T15, A6)
    print "(n, a, o, p)"
    print "--------------------------"
    print T16
    print ""
    print "(x, y, z, phi, theta, phi)"
    # Calculate x, y, z, phi, theta, zho
    if T16[0, 1] != 0:
        phi = atan2(T16[1, 1], T16[0, 1])
        if phi < 0:
            phi += pi
        theta = atan2(cos(phi)*T16[0, 1] + sin(phi)*T16[1, 1], T16[2, 1])
        zho = atan2(-sin(phi)*T16[0, 0] + cos(phi)*T16[0, 1], -sin(phi)*T16[2, 0] + cos(phi)*T16[2, 1])
    print "--------------------------"
    print "  ", format(T16[0, 3], '.4f'), "  ", format(T16[1, 3], '.4f'), "  ", format(T16[2, 3], '.4f'), "  ", format(phi, '.4f'), "  ", format(theta, '.4f'), "  ", format(zho, '.4f'),

    print "--------------------------"
    print ""

    nx = T16[0, 0]
    ny = T16[1, 0]
    nz = T16[2, 0]
    ox = T16[0, 1]
    oy = T16[1, 1]
    oz = T16[2, 1]
    ax = T16[0, 2]
    ay = T16[1, 2]
    az = T16[2, 2]
    px = T16[0, 3]
    py = T16[1, 3]
    pz = T16[2, 3]

    # Calculate ik
    # for fun
    inverse_kinematics(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz)

def main(mode_flag):

    if mode_flag == 1:
        theta_1 = input("theta_1: ")
        theta_1 = radians(theta_1)
        theta_2 = input("theta_2: ")
        theta_2 = radians(theta_2)
        theta_3 = input("theta_3: ")
        theta_3 = radians(theta_3)
        theta_4 = input("theta_4: ")
        theta_4 = radians(theta_4)
        theta_5 = input("theta_5: ")
        theta_5 = radians(theta_5)
        theta_6 = input("theta_6: ")
        theta_6 = radians(theta_6)

        kinematics(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)

    elif mode_flag == 2:
        nx = input("nx: ")
        ny = input("ny: ")
        nz = input("nz: ")
        ox = input("ox: ")
        oy = input("oy: ")
        oz = input("oz: ")
        ax = input("ax: ")
        ay = input("ay: ")
        az = input("az: ")
        px = input("px: ")
        py = input("py: ")
        pz = input("pz: ")

        mat = np.array([[nx, ox, ax], [ny, oy, ay], [nz, oz, az], [0, 0, 0, 1]])

        inverse_kinematics(nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz)

        return 0

if __name__ == "__main__":

    while True:
        mode = input("Calculate kinematics(1) or inverse kinematics(2) or quit(3)? (1 or 2 or 3) ")
        if mode == 3:
            break
        else:
            main(mode)

    print "I love TAs. Please give a good score. TAT"
