# -*- coding: utf-8 -*-
import open3d as o3d
from copy import deepcopy

def mesh(file_path='./fusioned_pcd.pcd'):
    pcd = o3d.io.read_point_cloud(file_path)
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    print(pcd)

    pcd1 = deepcopy(pcd)
    pcd1.translate((200, 0, 0))
    mesh1 = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd1, alpha=5)
    mesh1.paint_uniform_color([0, 1, 0])
    print(mesh1)

    o3d.visualization.draw_geometries([mesh1],
                                      window_name="点云重建",
                                      point_show_normal=False,
                                      mesh_show_wireframe=True,
                                      mesh_show_back_face=True,
                                      )
    o3d.io.write_triangle_mesh('./mesh.ply',mesh1)
# mesh()