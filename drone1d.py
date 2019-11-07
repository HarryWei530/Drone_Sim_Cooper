import matplotlib
import sys
import numpy as np
import scipy
import control
import pylab
from math import sqrt

#Constants/Parameters
g = -9.81
m = 0.33
d = 39.73*10**-3
CD = 7.9379*10**-12
CT = 3.1582*10**-10
Ixx = 1.395*10**-5
Iyy = 1.436*10**-5
Izz = 2.173*10**-5
Kp = 1

#State Space
A = np.array([[0,0,0,0,0,0,1,0,0,0,0,0],
              [0,0,0,0,0,0,0,1,0,0,0,0],
              [0,0,0,0,0,0,0,0,1,0,0,0],
              [0,0,0,0,0,0,0,0,0,1,0,0],
              [0,0,0,0,0,0,0,0,0,0,1,0],
              [0,0,0,0,0,0,0,0,0,0,0,1],
              [0,0,0,0,0,g,0,0,0,0,0,0],
              [0,0,0,0,0,0,-g,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0],
              ])

B = np.array([[0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [2*CT/m, 2*CT/m, 2*CT/m, 2*CT/m],
              [2*CT/m, 2*CT/m, 2*CT/m, 2*CT/m],
              [-sqrt(2)*d*CT/Iyy, sqrt(2)*d*CT/Iyy, sqrt(2)*d*CT/Iyy, -sqrt(2)*d*CT/Iyy],
              [-sqrt(2)*d*CT/Iyy, sqrt(2)*d*CT/Iyy, sqrt(2)*d*CT/Iyy, -sqrt(2)*d*CT/Iyy],
              ])


C = np.eye(12)


D = np.zeros((12,4))


sys = control.StateSpace(A,B,C,D)
sys_tf = control.tf(sys)

# print(sys)
print(sys_tf)


controller = Kp

L = sys_tf * controller
G = []
for i in range(0,np.size(L)):
    G[i] = sys_tf[i]

print(G)


# G = np.divide(L, 1 + L)

T, yout = control.step_response(G)


# PWM = np.array([1, 1, 5, 5])
# K = 0.2685
# RPM = K * PWM + 4070.3

# T, yout1 = control.step_response(sys, input=0)
# T, yout2 = control.step_response(sys, input=1)
# T, yout3 = control.step_response(sys, input=2)
# T, yout4 = control.step_response(sys, input=3)

# yout = yout1 + yout2 + yout3 + yout4

# print(T)
# print(yout)
# print(np.shape(T))
# print(np.shape(yout[0,:]))


pylab.plot(T, yout[0,:], 'g', linewidth=1)
# pylab.plot(T, yout[1,:], 'b', linewidth=1)
# pylab.plot(T, yout[2,:], 'r', linewidth=1)
pylab.show()