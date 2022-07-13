#coding=gbk
import numpy as np
import open3d as o3d
import os
from tqdm import tqdm

def get_pcd(data_path):
    #此处参数为相机内参，wh为图像宽高
    camera_factor = 1

    # #sr300
    # camera_cx = 313
    # camera_cy = 245.245
    #
    # camera_fx = 476.623
    # camera_fy = 476.623

    #d455
    camera_cx = 321.665
    camera_cy = 245.667

    camera_fx = 393.823
    camera_fy = 393.823

    w = 320*2
    h = 240*2
    data = np.load(data_path)
    points = np.zeros((w * h, 3), dtype=np.float32)
    n = 0
    for i in range(h):
        for j in range(w):
            deep = data[i,j]
            points[n][2] = deep / camera_factor
            points[n][0] = (j - camera_cx) * points[n][2] / camera_fx
            points[n][1] = (i - camera_cy) * points[n][2] / camera_fy

            n = n + 1

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd


def get_all_pcd(depth_file='./depth',pcd_file='./pcd'):
    for filepath,dirnames,filenames in os.walk(depth_file):
        for filename in tqdm(filenames):
            pcd = get_pcd(filepath+'/'+filename)
            # o3d.visualization.draw_geometries([pcd])
            o3d.io.write_point_cloud(pcd_file+'/'+filename[:-3]+'pcd',pcd)
            print(pcd_file+'/'+filename[:-3]+'pcd')


# get_all_pcd('./depth','./pcd')