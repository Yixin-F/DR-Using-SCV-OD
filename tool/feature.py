import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

x1 = np.arange(1,45,9)
x2 = x1+1
x3 = x2+1
x4 = x3+1
x5 = x4+1
x6 = x5+1
x7 = x6+1

label = ['Planarity', 'Linearity', 'Scattering ','Orientation','Max Height','Min Height','Scale']
name =['Building', 'Tree', 'Car', 'Cyclist', 'Pedestrian']

#builds     tree     cars        bicylist          people
pl = [[1.025, 0.9986, 0.8886, 0.8126], [0.1563,0.0896,0.1111,0.1236], [0.3456,0.4417,0.4012,0.3698], [0.2356, 0.2125,0.1589,0.1269],[0.1023,0.0745,0.1212,0.0096]]
ln = [[0.1256,0.0356,0.0893,0.0459], [0.7856,0.6359,0.5930,0.6698], [0.2365,0.1963,0.1125,0.1596], [0.2696,0.3012,0.3156,0.1930], [0.4569,0.6023,0.5569,0.5236]]
sc = [[0.1263,0.0869,0.1698,0.1548], [0.6854,0.4654,0.5967,0.7687],[0.1698,0.2020,0.3030,0.2569],[0.1253,0.0968,0.1639,0.1456],[0.0698,0.1023,0.0869,0.1123]]
po = [[0.2356,0.3012,0.1859,0.2798], [0.756,0.8564,0.8326,0.6986], [0.3698,0.4123,0.5263,0.5789],[0.4585,0.3659,0.5689,0.5023], [0.8569,0.7965,0.9232,1.0235]]
max = [[0.6857, 0.5367,0.5026,0.4869], [0.6845,0.7895,0.6245,0.8256], [0.5236,0.4586,0.4023,0.3654],[0.3256,0.3029,0.2869,0.2786], [0.4256,0.3869,0.3564,0.4026]]
min =[[0.0589,0.0426,0.0869,0.0659], [0.1234,0.0689,0.0897,0.0956], [0.0968,0.0756,0.0635,0.1023], [0.1236,0.0986,0.07456,0.0898], [0.1103,0.1023,0.0986,0.0856]]
le = [[0.756,0.865,1.036,0.968], [0.6535,0.5635,0.6023,0.4896], [0.4268,0.3698,0.4021,0.3987], [0.2659,0.3012,0.2265,0.2031], [0.1235,0.0986,0.1564,0.1356]]


plt.grid(axis='y',ls='--',alpha=0.8)
box1 = plt.boxplot(pl,positions=x1,patch_artist=True,showmeans=True,
            boxprops={"facecolor": "C0",
                      "edgecolor": "grey",
                      "linewidth": 0.5},
            medianprops={"color": "k", "linewidth": 0.5},
            meanprops={'ls':'--',
                       'markerfacecolor':'r',
                       'markeredgecolor':'r',
                       'markersize':4})

box2 = plt.boxplot(ln,positions=x2,patch_artist=True,showmeans=True,
            boxprops={"facecolor": "C1",
                      "edgecolor": "grey",
                      "linewidth": 0.5},
            medianprops={"color": "k", "linewidth": 0.5},
            meanprops={'ls':'--',
                       'markerfacecolor':'r',
                       'markeredgecolor':'r',
                       'markersize':4})
box3 = plt.boxplot(sc,positions=x3,patch_artist=True,showmeans=True,
            boxprops={"facecolor": "C2",
                      "edgecolor": "grey",
                      "linewidth": 0.5},
            medianprops={"color": "k", "linewidth": 0.5},
            meanprops={'ls':'--',
                       'markerfacecolor':'r',
                       'markeredgecolor':'r',
                       'markersize':4})
box4 = plt.boxplot(po,positions=x4,patch_artist=True,showmeans=True,
            boxprops={"facecolor": "C3",
                      "edgecolor": "grey",
                      "linewidth": 0.5},
            medianprops={"color": "k", "linewidth": 0.5},
            meanprops={'ls':'--',
                       'markerfacecolor':'r',
                       'markeredgecolor':'r',
                       'markersize':4})
box5 = plt.boxplot(max,positions=x5,patch_artist=True,showmeans=True,
            boxprops={"facecolor": "C4",
                      "edgecolor": "grey",
                      "linewidth": 0.5},
            medianprops={"color": "k", "linewidth": 0.5},
            meanprops={'ls':'--',
                       'markerfacecolor':'r',
                       'markeredgecolor':'r',
                       'markersize':4})
box6 = plt.boxplot(min,positions=x6,patch_artist=True,showmeans=True,
            boxprops={"facecolor": "C5",
                      "edgecolor": "grey",
                      "linewidth": 0.2},
            medianprops={"color": "k", "linewidth": 0.5},
            meanprops={'ls':'--',
                       'markerfacecolor':'r',
                       'markeredgecolor':'r',
                       'markersize':4})
box7 = plt.boxplot(le,positions=x7,patch_artist=True,showmeans=True,
            boxprops={"facecolor": "C6",
                      "edgecolor": "grey",
                      "linewidth": 0.5},
            medianprops={"color": "k", "linewidth": 0.5},
            meanprops={'ls':'--',
                       'markerfacecolor':'r',
                       'markeredgecolor':'r',
                       'markersize':4})

plt.xticks([4,13,22,31,40],name,fontsize=11)
plt.xlim(0,45)
plt.ylim(0,1.2)
plt.ylabel('Geo. Feature Value',fontsize=11)
plt.legend(handles=[box1['boxes'][0],box2['boxes'][0],box3['boxes'][0],box4['boxes'][0],box5['boxes'][0],box6['boxes'][0],box7['boxes'][0]],labels=label,loc = 'lower left', bbox_to_anchor = (0.6, 0.6))
plt.tight_layout()
plt.show()