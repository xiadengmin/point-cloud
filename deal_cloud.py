#coding=gbk
import open3d as o3d
import os
import numpy as np
from tqdm import tqdm

def hard_delete(file,hard_value_z,hard_value_y):
    for filepath,dirnames,filenames in os.walk(file):
        for filename in tqdm(filenames):
            pcd = o3d.io.read_point_cloud(filepath + '/' + filename)
            pcd_points = np.asarray(pcd.points)
            lst = []
            for i in range(307200):
                # print(pcd_points[i,1])
                if pcd_points[i, 2] < hard_value_z and pcd_points[i, 1] < hard_value_y:
                    lst.append(pcd_points[i, :])

            lst = np.array(lst)
            pcd.points = o3d.utility.Vector3dVector(lst)
            o3d.io.write_point_cloud("./hard_value_pcd/" + filename, pcd)

def deal(file):
    for filepath,dirnames,filenames in os.walk(file):
        for filename in tqdm(filenames):
            pcd = o3d.io.read_point_cloud(filepath + '/' + filename)
            dis = 10
            rnn = 10
            n = 1000
            plane_model, inliers = pcd.segment_plane(distance_threshold=dis, ransac_n=rnn, num_iterations=n)
            pcd_out = pcd.select_by_index(inliers, invert=True)
            o3d.io.write_point_cloud("./dealed_pcd/" + filename, pcd_out)

# hard_delete('./pcd',1200,130)
# deal('./pcd')