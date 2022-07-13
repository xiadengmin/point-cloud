#coding=gbk
import numpy as np
import open3d as o3d
import cv2
#显示PCD点云

def show_pcd(path="./pcd/1.pcd"):
    pcd = o3d.io.read_point_cloud(path)
    print(pcd)
    o3d.visualization.draw_geometries([pcd])

def show_hard_value_pcd(path="./hard_value_pcd/1.pcd"):
    pcd = o3d.io.read_point_cloud(path)
    print(pcd)
    o3d.visualization.draw_geometries([pcd])

def show_fusion_point_image(path='./fusioned_pcd.pcd'):
    pcd = o3d.io.read_point_cloud(path)
    print(pcd)
    o3d.visualization.draw_geometries([pcd])

def show_depth(path='./depth/1.npy'):
    depth_image = np.load(path)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    cv2.imshow('depth',depth_colormap)
    cv2.waitKey()

def show_mesh(path="./mesh.ply"):
    mesh = o3d.io.read_triangle_mesh(path)
    o3d.visualization.draw_geometries([mesh],
                                      window_name="点云重建",
                                      point_show_normal=False,
                                      mesh_show_wireframe=True,
                                      mesh_show_back_face=True,
                                      )


def show_numpy(path='./depth/1.npy'):
    depth_image = np.load(path)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    print(depth_image)
    print(depth_image[0][0]*1000000)
    # 285 230 285 80
    # cv2.imshow('depth',depth_colormap)
    cv2.waitKey()

# show_depth()
# show_pcd('./dealed_pcd/100.pcd')
# show_numpy()
# show_mesh()