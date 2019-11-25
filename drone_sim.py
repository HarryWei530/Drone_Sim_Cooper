import matplotlib.pyplot as plt
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


plant = control.StateSpace(A,B,C,D)


# Rate Controller 
Kp = 2
Ki = 1
omega = 100
rate_roll_ctrl = control.tf([Kp,Ki],[1,0])
rate_pitch_ctrl = control.tf([0,0],[1,0])
rate_yaw_ctrl = control.tf([0,0],[1,0])

PWM1 = omega - rate_pitch_ctrl/2 + rate_roll_ctrl/2 - rate_yaw_ctrl
PWM2 = omega + rate_pitch_ctrl/2 - rate_roll_ctrl/2 + rate_yaw_ctrl
PWM3 = omega + rate_pitch_ctrl/2 + rate_roll_ctrl/2 - rate_yaw_ctrl
PWM4 = omega - rate_pitch_ctrl/2 - rate_roll_ctrl/2 + rate_yaw_ctrl

print(PWM1)
print(PWM2)
print(PWM3)
print(PWM4)

w1 = control.tf([54.237, 8140.8685],[2,0])
w2 = control.tf([53.163, 8140.3315],[2,0])
w3 = control.tf([54.237, 8140.8685],[2,0])
w4 = control.tf([53.163, 8140.3315],[2,0])

s = control.tf ([1,0],[1])



# A1 = np.zeros((3,3))

# B1 = np.zeros((3,1))

# C1 = np.array(
#     [Kp,Kp,Kp]
# )

# D1 = np.zeros((1,1))

# rate_ctrl = control.StateSpace(A1,B1,C1,D1)
T, yout1 = control.step_response(w1)
T, yout2 = control.step_response(w2)
T, yout3 = control.step_response(w3)
T, yout4 = control.step_response(w4)

# plant_tf = control.tf(plant)

# result = np.shape(plant_tf.num)
# rows = result[0]
# cols = result[1]


# Define Kp, Ki, Kd controllers, constants defined at beginning of code
# p_tf = control.tf([Kp],[1])
# I_tf = control.tf([Ki],[1,0])
# D_tf = control.tf([Kd,0],[1])

# controller_tf = control.parallel(p_tf,I_tf,D_tf)

# L_tf = [[0 for i in range(cols)] for j in range(rows)] # initialize zero 2D list

# for row in range(0,rows):
#     for col in range(0,cols):
#         curplant_tf = control.tf(plant_tf.num[row][col],plant_tf.den[row][col])
#         L_tf[row][col] = control.series(controller_tf,curplant_tf)

# G = control.feedback(L_tf[0][0],1)


# print(controller_tf)
# for i in range(0,tf_size):
    # T[i], yout[i] = control.step_response(L_tf, input=i)

#Plot step response of plant, summing up all transfer function outputs
# T, yout = control.step_response(G, input=0)
# # PLot step response of plant, input = 0,1,2,3 steps through one motor at a time
# T, yout1 = control.step_response(plant, input=0)
# T, yout2 = control.step_response(plant, input=1)
# T, yout3 = control.step_response(plant, input=2)
# T, yout4 = control.step_response(plant, input=3)

# yout = yout1 + yout2 + yout3 + yout4

# # print(T)
# # print(yout)
# # print(np.shape(T))
# # print(np.shape(yout[0,:]))

pylab.plot(T, yout1, 'g', linewidth=1, marker="*")
pylab.plot(T, yout2, 'r', linewidth=1, marker="*")
pylab.plot(T, yout3, 'b', linewidth=1)
pylab.plot(T, yout4, 'y', linewidth=1)
# # # pylab.plot(T, yout[1,:], 'b', linewidth=1)
# # # pylab.plot(T, yout[2,:], 'r', linewidth=1)
pylab.show()






# # yout = np.array(0)


# print(t)
# # print(x)
# print(yout)
# # print(np.shape(resp))
# # plt.plot(resp)

# # print(plant)
# # print(plant_tf)


# L_tf = control.tf(Kp,1)

# L = control.series(plant_tf,L_tf)
# # plant_tf * controller


# # for i in range(0,np.size(L)):
# #     G[i] = plant_tf[i]

# # print(G)


# # G = np.divide(L, 1 + L)

# T, yout = control.step_response(G)


# # PWM = np.array([1, 1, 5, 5])
# # K = 0.2685
# # RPM = K * PWM + 4070.3

