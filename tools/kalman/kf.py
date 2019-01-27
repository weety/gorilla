# -*- coding: utf-8 -*-
"""
Created on Tue Nov 27 08:52:36 2018

@author: weety
"""

import numpy as np
import matplotlib.pyplot as plt

PI = 3.1415926

# read data from file
file = open("sensor_data_delta2.txt","r")
list_arr = file.readlines()
l = len(list_arr)
for i in range(l): 
    list_arr[i] = list_arr[i].split() 
data = np.array(list_arr)
data = data.astype(float)
#print (data)
file.close()

acc = data[:, 0:3]
gyr = data[:, 3:6]

dt = 0.01

A = np.mat([[1, -dt], [0,1]])
B = np.mat([[dt], [0]])
Q = np.mat([[0.00001, 0], [0, 0.00003]])
R = np.mat([0.8])
H = np.mat([1,0])
I = np.mat(np.eye(2,2))

Gyro = gyr[:,0]
zk = np.arctan(acc[:,1]/np.sqrt(acc[:,0]**2 + acc[:,2]**2))

X=np.mat([[0], [0]])
P=np.mat([[1, 0], [0, 1]])
K=np.mat([[0], [0]])

data_len = len(acc)
data_range = range(data_len)

Angle = np.zeros(data_len)
Qbias = np.zeros(data_len)
Gyro_final = np.zeros(data_len)

#kalman filter
for i in data_range:
    #X(k|k-1) = A * X(k-1|k-1) + B*u(k)
    X = A * X + B * (Gyro[i] - X[1])
    #P(k|k-1) = A * P(k-1|k-1) * AT + Q
    P = A * P * A.T + Q
    #K(k) = P(k|k-1) * HT / (H * P(k|k-1) * HT + R)
    K = P * H.T * np.linalg.pinv(H * P * H.T + R) 
    #e = Z(k)- H * X(k|k-1)
    #X(k|k) = X(k|k-1) + K * e = X(k|k-1) + K * (Z(k) - H * X(k|k-1))
    X = X + K * (zk[i] - H * X)
    #P(k|k) = P(k|k-1) - K * H * P(k|k-1) = (I - K * H) * P(k|k-1)
    P = (I - K * H) * P
    #Y(k) = H * X(k|k)
    Angle[i] = X[0]
    Qbias[i] = X[1]
    Gyro_final[i] = Gyro[i] - X[1]

#draw wave picture
plt.figure()
plt.plot(data_range, Angle*180/PI, label="Angle",color="red",linewidth=2)
plt.plot(data_range, zk*180/PI, "g--", label="Angle_acc")
plt.xlabel("Time(ms)")
plt.ylabel("Angle(°)")
plt.title("Angle filter")
plt.legend()
plt.show()
plt.figure()
plt.plot(data_range, Gyro_final, "b", label="Gyro_final")
plt.plot(data_range, Gyro, "r--", label="Gyro")
plt.plot(data_range, Qbias, "g--", label="Qbias")
plt.xlabel("Time(ms)")
plt.ylabel("Angular velocity(rad)")
plt.title("Angle velocity filter")
plt.legend()
plt.show()
    
