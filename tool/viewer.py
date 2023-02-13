import open3d as o3d
import numpy as np


# -------------------------加载点云-----------------------------
pcd = o3d.io.read_point_cloud('/home/fyx/ufo_hiahia/src/out/seg/10_seg.pcd')
# -----------------------初始化显示窗口--------------------------
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='可视化', width=800, height=600)
# -----------------------可视化参数设置--------------------------
opt = vis.get_render_option()
opt.background_color = np.asarray([255, 255, 255])  # 设置背景色*****
opt.point_size = 2                  # 设置点的大小*************
# opt.show_coordinate_frame = True    # 设置是否添加坐标系
pcd.paint_uniform_color([0, 0, 1])  # 自定义点云显示颜色
vis.add_geometry(pcd)               # 加载点云到可视化窗口
vis.run()                           # 激活显示窗口，这个函数将阻塞当前线程，直到窗口关闭。
vis.destroy_window()                # 销毁窗口，这个函数必须从主线程调用。

