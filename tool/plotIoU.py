import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as mtick  

# x = ['Ground', 'Building', 'Vegetation', 'PD']
# name = ['Ori', 'Ori-RPC', 'Ori-TC', 'Ori-RPC+TC']
# color4 = ['red', 'green', 'brown', 'gold']
# marker4 = ['x', 'o', 'd', 'v']

# num00 = [678, 472, 997, 178]
# iou00 = [[98.51, 40.26, 50.56, 91.42], [98.51, 46.23, 57.02, 92.42], [98.51, 42.89, 52.44, 91.55], [98.51, 50.37, 55.89, 96.08]]

# num01 = [513, 195, 1093, 31]
# iou01 = [[97.26, 47.34, 53.16, 92.91], [97.26, 49.10, 55.99, 94.28], [97.26, 46.20, 54.27, 93.38], [97.26, 50.10, 58.45, 96.13]]

# num02 = [642, 198, 820, 35]
# iou02 = [[96.23, 44.42, 47.39, 92.23], [96.23, 50.73, 51.64, 95.07], [96.23, 47.12, 46.59, 94.22], [96.23, 47.10, 52.22, 98.62]]

# num05 = [1069, 776, 1126, 215]
# iou05 = [[98.27, 49.69, 50.12, 92.77], [98.27, 50.97, 51.47, 95.12], [98.27, 48.23, 52.18, 93.66], [98.27, 54.69, 55.96, 97.25]]

# num07 = [766, 712, 288, 234]
# iou07 = [[95.91, 47.25, 45.29, 90.58], [95.91, 51.02, 50.47, 95.29], [95.91, 48.11, 49.94, 92.36], [95.91, 54.82, 48.26, 98.11]]

# fig = plt.figure()  
# ax1 = fig.add_subplot(111)  
# plt.bar(x, num07, alpha = 0.4, width = 0.3, color = 'purple')   # TODO:
# ax1.legend(['Ground truth'], loc = 'upper left', bbox_to_anchor = (0.15, 1.0))
# ax1.set_ylabel("Points number [k]")
# ax1.set_ylim([0, 1200])

# ax2 = ax1.twinx()
# for i in range(len(iou01)):  
#     plt.plot(x, iou07[i], marker = marker4[i], linewidth = 0.8, linestyle = '-.')  #TODO:
# ax2.legend(name, loc = 'upper right', bbox_to_anchor = (1.0, 0.45))
# ax2.set_ylim([30, 100])
# ax2.set_ylabel('IoU [%]')

# plt.grid(True, linestyle = ":", color = 'black', alpha = 0.5)
# plt.title("Sequence 07")   # TODO:
# plt.show()

# labels = ['00', '01', '02', '05', '07']
# ground = [98.51, 97.26, 96.23, 98.27, 95.91]
# buildings = [50.37, 50.10, 47.10, 54.69, 54.82]
# vegetation = [55.89, 58.45, 52.22, 55.96, 48.26]
# PD = [96.08, 96.13, 98.62, 97.25, 98.11]

# x = np.arange(len(labels))  # the label locations
# width = 0.2  # the width of the bars

# fig, ax = plt.subplots()

# rects1 = ax.bar(x - width*1.5, ground, width, label='Ground', color='#2878b5')
# rects2 = ax.bar(x + width*0.5, buildings, width, label='Buildings', color='#f8ac8c')
# rects3 = ax.bar(x + width*1.5, vegetation, width, label='Vegetation', color='#9ac9db')
# rects4 = ax.bar(x - width*0.5, PD, width, label='PD', color='#ff8884')

# ax.set_ylabel('IOU [%]')
# ax.set_xlabel('Sequence')
# ax.set_ylim([20, 100])
# # ax.set_title('IOU of various categories')
# ax.set_xticks(x)
# ax.set_xticklabels(labels)
# ax.legend(['Ground', 'Buildings', 'Vegetation', 'PD'], loc = 'upper right', bbox_to_anchor = (1.0, 0.9))
# plt.grid(True, linestyle = ":", color = 'black', alpha = 0.5)
# # plt.title("IOU of Various Categories")   # TODO:
# plt.show()

labels = ['00', '01', '02', '05', '07']
lg= [35, 0, 4, 34, 65]
lr = [32, 0, 4, 32, 58]
hg = [40, 102, 37, 67, 61]
hr = [37, 96, 32, 60, 53]

x = np.arange(len(labels))  # the label locations
width = 0.2  # the width of the bars

fig, ax = plt.subplots()

rects1 = ax.bar(x - width*1.5, lg, width, label='HD_gt', color='#32B897')
rects2 = ax.bar(x - width*0.5, lr, width, label='HD_removed', color='#05B9E2')
rects3 = ax.bar(x + width*0.5, hg, width, label='LD_gt', color='#8983BF')
rects4 = ax.bar(x + width*1.5, hr, width, label='LD_retained', color='#C76DA2')

ax.set_ylabel('Object Number')
ax.set_xlabel('Sequence')
ax.set_ylim([0, 120])
# ax.set_title('IOU of various categories')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend(['HD_gt', 'HD_removed', 'LD_gt', 'LD_retained'], loc = 'upper right', bbox_to_anchor = (1.0, 1.0))
plt.grid(True, linestyle = ":", color = 'black', alpha = 0.5)
# plt.title("IOU of Various Categories")   # TODO:
plt.show()