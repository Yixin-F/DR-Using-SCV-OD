# import os
# import numpy as np
# import sys

# def read_pcd(filepath):
#     lidar = []
#     with open(filepath,'r') as f:
#         line = f.readline().strip('\n')
#         while line:
#             linestr = line.split(" ")
#             if len(linestr) == 4:
#                 linestr_convert = []
#                 for l in linestr:
#                     linestr_convert.append(float(l))
#                 lidar.append(linestr_convert)
#     return np.array(lidar)


# def convert(pcdfolder, binfolder):
#     # current_path = os.getcwd()
#     # ori_path = os.path.join(current_path, pcdfolder)
#     # file_list = os.listdir(ori_path)
#     # des_path = os.path.join(current_path, binfolder)
#     ori_path = pcdfolder
#     file_list = os.listdir(ori_path)
#     des_path = binfolder
#     if os.path.exists(des_path):
#         pass
#     else:
#         os.makedirs(des_path)
#     for file in file_list: 
#         (filename,extension) = os.path.splitext(file)
#         velodyne_file = os.path.join(ori_path, filename) + '.pcd'
#         print(filename);
#         pl = read_pcd(velodyne_file)
#         pl = pl.reshape(-1, 4).astype(np.float32)
#         velodyne_file_new = os.path.join(des_path, filename) + '.bin'
#         pl.tofile(velodyne_file_new)
    
# if __name__ == "__main__":
#     pcdfolder = sys.argv[2]
#     binfolder = sys.argv[3]
#     convert(pcdfolder, binfolder)

import os
import numpy as np
import fire

def read_pcd(filepath):
    lidar = []
    with open(filepath,'r') as f:
        line = f.readline().strip()
        while line:
            linestr = line.split(" ")
            if len(linestr) == 4:
                linestr_convert = list(map(float, linestr))
                lidar.append(linestr_convert)
            line = f.readline().strip()
    return np.array(lidar)


def convert(pcdfolder, binfolder):
    current_path = os.getcwd()
    ori_path = os.path.join(current_path, pcdfolder)
    file_list = os.listdir(ori_path)
    des_path = os.path.join(current_path, binfolder)
    if os.path.exists(des_path):
        pass
    else:
        os.makedirs(des_path)
    
    i = 0
    for file in file_list: 
        (filename,extension) = os.path.splitext(file)
        velodyne_file = os.path.join(ori_path, filename) + '.pcd'
        pl = read_pcd(velodyne_file)
        pl = pl.reshape(-1, 4).astype(np.float32)
        velodyne_file_new = os.path.join(des_path, str(i)) + '.bin'
        print(i)
        i=i+1
        pl.tofile(velodyne_file_new)
    
if __name__ == "__main__":
    fire.Fire()    
