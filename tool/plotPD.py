import matplotlib.pyplot as plt
import numpy as np

# name = ['00', '01', '02', '05', '07']

# pr = [[94.23, 98.81, 99.02], [88.64, 94.38, 96.21], [94.47, 98.52, 98.96], [92.61, 98.97, 99.46], [92.88, 97.74, 98.69]]
# rr = [[97.47, 94.53, 90.26], [96.50, 94.65, 91.39], [99.11, 96.12, 89.82], [98.73, 96.67, 92.23], [99.07, 97.66, 91.48]]

# x = [0.2, 0.5, 0.8]

color = ['red', 'blue', 'green', 'brown', 'gold']
marker = ['s', 'v', 'd', 'o', 'x']

# x_ticks = np.arange(0.2, 0.8, 0.3)

# for i in range(len(name)):
#     plt.plot(x, rr[i], color = color[i], marker = marker[i], linewidth = 0.8, linestyle = "-.")

# plt.legend(name, loc = 'lower left', bbox_to_anchor = (0.84, 0.0)) # 0.84
# plt.xticks(x_ticks)
# plt.xlabel('object overlap ratio threshold')
# plt.ylabel('RR [%]')
# plt.title('Rejection Rate')
# plt.grid(True, linestyle = ":", color = 'black', alpha = 0.5)
# plt.show()

name = ['BA', 'BA - RPC', 'BA - TC', 'BA - RPC+TC']
x = ['00', '01', '02', '05', '07']
title1 = 'Perservation Rate (PR)'
title2 = 'Rejection Rate (RR)'
title3 = 'F1 Score'
title4 = 'Potential Dynamic Object  IOU'
y1 = [[94.454, 90.011, 92.278, 94.288, 90.487], [96.881, 91.874, 96.132, 96.517, 94.417], [96.868, 89.827, 96.098, 96.511, 94.684], [98.821, 94.388, 98.521, 98.972, 94.747]]
y2 = [[90.129, 91.269, 89.927, 90.256, 91.299], [90.979, 93.420, 92.983, 91.421, 92.677], [89.100, 93.204, 90.481, 91.423, 93.568], [95.242, 94.257, 96.127, 96.674, 98.666]]
y3 = [[0.922, 0.906, 0.911, 0.922, 0.909], [0.938, 0.926, 0.945, 0.939, 0.935], [0.928, 0.915, 0.932, 0.932, 0.941], [0.970, 0.943, 0.973, 0.978, 0.967]]
y3 = [[91.42, 92.91, 92.23, 92.77, 90.58], [92.42, 94.28, 95.07, 95.12, 95.29], [91.55, 93.38, 94.22, 93.66, 92.36], [96.08, 96.13, 98.62, 97.25, 98.11]]

for i in range(len(name)):
    plt.plot(x, y1[i], color = color[i], marker = marker[i], linewidth = 0.8, linestyle = "-.")

plt.legend(name, loc = 'lower left', bbox_to_anchor = (0.84, 0.0)) # 0.84
# plt.xticks(x_ticks)
plt.xlabel('Sequence Name')
plt.ylabel('PR [%]')
plt.title(title1)
plt.grid(True, linestyle = ":", color = 'black', alpha = 0.5)
plt.show()