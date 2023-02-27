import pypcd
from tqdm import tqdm
import numpy as np
from sklearn.neighbors import NearestNeighbors
from tabulate import tabulate
import pclpy
from pclpy import pcl
DYNAMIC_CLASSES = [252, 253, 254, 255, 256, 257, 258, 259]

obj = pclpy.pcl.PointCloud.PointXYZRGB()

def intensity2labels(intensity_np):
    label = intensity_np.astype(np.uint32)
    sem_label = label & 0xFFFF  # semantic label in lower half
    inst_label = label >> 16  # instance id in upper half
    return sem_label, inst_label

def load_pcd(path):
    print("On loading data...")
    data = pypcd.PointCloud.from_path(path)
    print("Complete to load data")
    return data

def determine_dynamic(intensity_np):
    sem_label, inst_label = intensity2labels(intensity_np)
    for class_id in DYNAMIC_CLASSES:
        if class_id == sem_label:
            return True
