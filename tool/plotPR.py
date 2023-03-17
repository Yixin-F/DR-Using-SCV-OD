import matplotlib.pyplot as plt

name = ['00', '01', '02', '05', '07']

pr = [[94.23, 98.81, 99.02], [88.64, 94.38, 96.21], [94.47, 98.52, 98.96], [92.61, 98.97, 99.46], [92.88, 97.74, 98.69]]
rr = [[97.47, 94.53, 90.26], [96.50, 94.65, 91.39], [99.11, 96.12, 89.82], [98.73, 96.67, 92.23], [99.07, 97.66, 91.48]]

x = [0.2, 0.5, 0.8]

color = ['red', 'blue', 'green', 'brown', 'gold']
marker = ['s', 'v', 'd', 'o', 'x']

for i in range(len(name)):
    plt.plot(x, rr[i], color = color[i], marker = marker[i], linewidth = 0.4, linestyle = "-.")

plt.legend(name, loc='lower left', bbox_to_anchor=(0.84, 0.0))

plt.show()