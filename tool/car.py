import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

names = ['t','intensity','id',
         'x','y','z',
         'azimuth','range','pid']

formats = ['int64', 'uint8', 'uint8',
           'float32', 'float32', 'float32',
           'float32', 'float32', 'int32']

binType = np.dtype( dict(names=names, formats=formats) )
data = np.fromfile('/home/fyx/sydney-urban-objects-dataset/objects/cyclist.0.7041.bin', binType)

# 3D points, one per row
P = np.vstack([ data['x'], data['y'], data['z'] ]).T


fig = plt.figure(figsize=(20,20))
ax = Axes3D(fig)
# print(P)

# #ax.plot3D(x,y,z)
ax.scatter3D(P[:,0],P[:,1],P[:,2],s=50,c='blue')
plt.show()