import matplotlib.pyplot as plt
import numpy as np
import matplotlib

av = []
cov = []

with open("/home/fyx/ufo_hiahia/src/out/calib/0_av.txt", "r") as f1:
    data1 = f1.read()
    for d1 in data1:
        # print(d1)
        av.append(d1)
f1.close()
print(len(av))

with open("/home/fyx/ufo_hiahia/src/out/calib/0_cov.txt", "r") as f2:
    data2 = f2.read()
    for d2 in data2:
        cov.append(d2)
f2.close()
print(len(cov))

# av = np.array(av)
# cov = np.array(cov)

plt.ylabel('value')
plt.xlabel('CVI index')
plt.hist(av, bins = 10)
plt.show()
