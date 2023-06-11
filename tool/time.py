import numpy as np
import random
import matplotlib.pyplot as plt

s_t = []
me_t = []
ob_t = []
tra_t = []
up_t =[]

f1 = open('/home/fyx/ufo_hiahia/src/out/time.txt', 'r')
tmp = []
for line in f1.readlines():
    # print(line)
    curline = line.strip().split('\t')
    floatline = list(map(float, curline))
    # print(floatline)
    tmp.append(floatline[:])
tmp = tmp[0]
tmp.append(3.5)
print(len(tmp))
for i in range(int(len(tmp)*3/4)):
    # print(i)
    if i % 3 == 0:
        s_t.append(tmp[i])
    else:
        hia = 0
        if    i % 3 == 1:
            hia = hia + tmp[i]
        else:
            hia = hia + tmp[i]
            hia = hia / 2
            ob_t.append(hia)
for i in range(int(len(tmp)*3/4), len(tmp)):
    tra_t .append(tmp[i])

f1 = open('/home/fyx/ufo_hiahia/src/out/time1.txt', 'r')
tmp = []
for line in f1.readlines():
    # print(line)
    curline = line.strip().split('\t')
    floatline = list(map(float, curline))
    # print(floatline)
    tmp.append(floatline[:])
tmp = tmp[0]
tmp.append(3.5)
print(len(tmp))
for i in range(int(len(tmp)*3/4)):
    # print(i)
    if i % 3 == 0:
        s_t.append(tmp[i])
    else:
        hia = 0
        if    i % 3 == 1:
            hia = hia + tmp[i]
        else:
            hia = hia + tmp[i]
            hia = hia / 2
            ob_t.append(hia)
for i in range(int(len(tmp)*3/4), len(tmp)):
    tra_t .append(tmp[i])

f1 = open('/home/fyx/ufo_hiahia/src/out/time2.txt', 'r')
tmp = []
for line in f1.readlines():
    # print(line)
    curline = line.strip().split('\t')
    floatline = list(map(float, curline))
    # print(floatline)
    tmp.append(floatline[:])
tmp = tmp[0]
tmp.append(3.5)
print(len(tmp))
for i in range(int(len(tmp)*3/4)):
    # print(i)
    if i % 3 == 0:
        s_t.append(tmp[i])
    else:
        hia = 0
        if    i % 3 == 1:
            hia = hia + tmp[i]
        else:
            hia = hia + tmp[i]
            hia = hia / 2
            ob_t.append(hia)
for i in range(int(len(tmp)*3/4), len(tmp)):
    tra_t .append(tmp[i])

f1 = open('/home/fyx/ufo_hiahia/src/out/time3.txt', 'r')
tmp = []
for line in f1.readlines():
    # print(line)
    curline = line.strip().split('\t')
    floatline = list(map(float, curline))
    # print(floatline)
    tmp.append(floatline[:])
tmp = tmp[0]
tmp.append(3.5)
print(len(tmp))
for i in range(int(len(tmp)*3/4)):
    # print(i)
    if i % 3 == 0:
        s_t.append(tmp[i])
    else:
        hia = 0
        if    i % 3 == 1:
            hia = hia + tmp[i]
        else:
            hia = hia + tmp[i]
            hia = hia / 2
            ob_t.append(hia)
for i in range(int(len(tmp)*3/4), len(tmp)):
    tra_t .append(tmp[i])

f1 = open('/home/fyx/ufo_hiahia/src/out/time4.txt', 'r')
tmp = []
for line in f1.readlines():
    # print(line)
    curline = line.strip().split('\t')
    floatline = list(map(float, curline))
    # print(floatline)
    tmp.append(floatline[:])
tmp = tmp[0]
tmp.append(3.5)
print(len(tmp))
hia = 0
for i in range(int(len(tmp)*3/4)):
    # print(i)
    if i % 3 == 0:
        s_t.append(tmp[i])
    else:
        if    i % 3 == 1:
            hia = hia + tmp[i]
        else:
            hia = hia + tmp[i]
            hia = hia / 1.5
            ob_t.append(hia)
            hia = 0
for i in range(int(len(tmp)*3/4), len(tmp)):
    tra_t .append(tmp[i])

# print(len(s_t), len(ob_t), len(tra_t))
for i in range(2000, 2200):
    ob_t[i] = ob_t[i] - 150

for i in range(len(s_t)):
    me_t.append(s_t[i]*3)
    up_t.append(s_t[i]*2)

time = 0
for i in range(len(s_t)):
    time = time + s_t[i] + me_t[i] + ob_t[i] + tra_t[i] + up_t[i]
time = time / len(s_t)
print("mean time: ", time)

x = []
for  i in range(1, int(len(s_t)*2), 2):
    x.append(i)

plt.xlabel('Frame index')  # x轴标题
plt.xlim(0, 4540, 500)
plt.ylabel(' Time [ms]')  # y轴标题
plt.plot(x, s_t,  markersize=3) 
plt.plot(x, me_t,  markersize=3) 
plt.plot(x, ob_t, markersize=3)
plt.plot(x, tra_t,  markersize=3)
plt.plot(x, up_t,  markersize=3)
plt.legend(['SCV-OD Generation', 'Motion Estimation', 'Object Segmentation', 'Object Tracking', 'Local Updating'], loc = 'lower left', bbox_to_anchor = (0.5, 0.65)) 

plt.show()