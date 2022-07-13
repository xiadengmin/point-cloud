# #coding=gbk
# import open3d as o3d
# import numpy as np
# import os
# from tqdm import tqdm
#
# def combine2pcd(target,source,i):
#     source = source.voxel_down_sample(voxel_size=0.2)
#     target = target.voxel_down_sample(voxel_size=0.2)
#
#     source.paint_uniform_color([0, 0, 1.0])
#     target.paint_uniform_color([0, 1, 0.0])
#
#     processed_source = source
#     processed_target = target
#     threshold = 1000  #变换阀值
#     trans_init = np.asarray([[1,0,0,0],
#                              [0,1,0,0],
#                              [0,0,1,0],
#                              [0,0,0,1]])
#
#     reg_p2p = o3d.pipelines.registration.registration_icp(
#             processed_source, processed_target, threshold, trans_init,
#             o3d.pipelines.registration.TransformationEstimationPointToPoint())
#
#     processed_source.transform(reg_p2p.transformation)
#     pcd_combined = o3d.geometry.PointCloud()
#     pcd_combined += target
#     pcd_combined += processed_source
#
#     # o3d.visualization.draw_geometries([pcd_combined])
#     pcd_combined = o3d.geometry.PointCloud.uniform_down_sample(pcd_combined,2)
#     # pcd_combined = pcd_combined.voxel_down_sample(voxel_size=0.5)
#     # print(pcd_combined)
#     return pcd_combined,reg_p2p.transformation
#
# def get_combined_pcd(file='./pcd'):
#     combined_pcd = o3d.io.read_point_cloud(file+'/'+'1.pcd')
#     for filepath,dirnames,filenames in os.walk(file):
#         for i in tqdm(range(len(filenames)-1)):
#             if i%100 ==0 and i!=0:
#                 o3d.visualization.draw_geometries([combined_pcd])
#
#             # print(filepath + '/' + str(i+2)+'.pcd')
#             pcd = o3d.io.read_point_cloud(filepath + '/' + str(i+2)+'.pcd')
#             combined_pcd,_ = combine2pcd(combined_pcd, pcd,i)
#             o3d.visualization.draw_geometries([combined_pcd])
#     o3d.io.write_point_cloud('./fusioned_pcd.pcd', combined_pcd)
#
# # def combine_pcd(file='./pcd'):
# #     combined_pcd = o3d.io.read_point_cloud(file+'/'+'1.pcd')
# #     firstpcd = o3d.io.read_point_cloud(file+'/'+'1.pcd')
# #     lst = []
# #     for filepath,dirnames,filenames in os.walk(file):
# #         for i in tqdm(range(len(filenames)-1)):
# #             trans_init = np.asarray([[1, 0, 0, 0],
# #                                      [0, 1, 0, 0],
# #                                      [0, 0, 1, 0],
# #                                      [0, 0, 0, 1]])
# #             # print(filepath + '/' + str(i+2)+'.pcd')
# #             secondpcd = o3d.io.read_point_cloud(filepath + '/' + str(i+2)+'.pcd')
# #             _,trans = combine2pcd(secondpcd,firstpcd)
# #             if len(lst)==0:
# #
# #                 firstpcd = firstpcd.transform(trans)
# #             else:
# #                 for trans in lst:
# #                     trans_init = np.dot(trans,trans_init)
# #                 firstpcd = firstpcd.transform(trans_init)
# #             combined_pcd+=firstpcd
# #             firstpcd = secondpcd
# #             if i%100 ==0:
# #                 o3d.visualization.draw_geometries([combined_pcd])
# #                 combined_pcd = o3d.geometry.PointCloud.uniform_down_sample(combined_pcd, 2)
# #     o3d.visualization.draw_geometries([combined_pcd])
# #     o3d.io.write_point_cloud('./fusioned_pcd.pcd', combined_pcd)
#
#
# get_combined_pcd('./dealing_pcd')
# # combine_pcd('./dealed_pcd')

import open3d as o3d
import numpy as np
import os
import deal_cloud

def load_point_clouds(voxel_size,pcd_name_lst):
    pcds = []
    for pcd_name in pcd_name_lst:
        pcd = o3d.io.read_point_cloud(pcd_name)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

def pairwise_registration(source, target,max_correspondence_distance_coarse,max_correspondence_distance_fine):
    # print("Apply point-to-plane ICP")
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(5))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(5))

    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp

def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id],max_correspondence_distance_coarse,max_correspondence_distance_fine)
            # print("Build o3d.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=True))
    return pose_graph

def get_all_pcd(pcd_file):
    lst = []
    little_lst = []
    num = 1
    for filepath, dirnames, filenames in os.walk(pcd_file):
        for i in range(len(filenames)):
            if num<=20:
                little_lst.append(filepath + '/' + str(i + 1) + '.pcd')
                num+=1
            else:
                num = 1
                lst.append(little_lst)
                little_lst = []
    print(lst)
    return lst


def get_dealing_data(pcd_file='./hard_value_pcd',max_correspondence_distance_coarse=0.3,max_correspondence_distance_fine=0.03):
    lst = get_all_pcd(pcd_file)

    i = 1
    for little_lst in lst:
        voxel_size = 0.02

        pcds_down = load_point_clouds(voxel_size,little_lst)
        # o3d.visualization.draw_geometries(pcds_down)

        # print("Full registration ...")
        pose_graph = full_registration(pcds_down,
                                       max_correspondence_distance_coarse,
                                       max_correspondence_distance_fine)
        # print("Transform points and display")
        for point_id in range(len(pcds_down)):
            # print(pose_graph.nodes[point_id].pose)
            pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
        # o3d.visualization.draw_geometries(pcds_down)
        pcds = load_point_clouds(voxel_size,little_lst)
        pcd_combined = o3d.geometry.PointCloud()
        for point_id in range(len(pcds)):
            pcds[point_id].transform(pose_graph.nodes[point_id].pose)
            pcd_combined += pcds[point_id]
        pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
        o3d.io.write_point_cloud('./dealing_pcd/'+str(i)+'.pcd', pcd_combined_down)
        i+=1
        # o3d.visualization.draw_geometries([pcd_combined_down])


def get_combined_pcd(pcd_file='./dealing_pcd',max_correspondence_distance_coarse=50,max_correspondence_distance_fine=10):
    deal_cloud.hard_delete('./pcd',1200,130)
    get_dealing_data()
    little_lst = []
    for filepath, dirnames, filenames in os.walk(pcd_file):
        for i in range(len(filenames)):
                little_lst.append(filepath + '/' + str(i + 1) + '.pcd')
    pcd_lst = little_lst

    voxel_size = 0.02
    pcds_down = load_point_clouds(voxel_size, pcd_lst)
    # o3d.visualization.draw_geometries(pcds_down)

    print("Full registration ...")
    pose_graph = full_registration(pcds_down,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine)
    print("Transform points and display")
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    # o3d.visualization.draw_geometries(pcds_down)
    pcds = load_point_clouds(voxel_size, pcd_lst)
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    pcd_lst.append(pcd_combined_down)
    # o3d.visualization.draw_geometries([pcd_combined_down])
    o3d.io.write_point_cloud('./fusioned_pcd.pcd', pcd_combined_down)


# get_combined_pcd()