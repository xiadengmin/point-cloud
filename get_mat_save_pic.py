#coding=gbk

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import datetime
import time

def tid_maker():
    return '{0:%Y%m%d%H%M%S%f}'.format(datetime.datetime.now())

print(tid_maker())
# 创建新文件夹及文件路径
depth_path='./depth/'+tid_maker()
color_path = './color/'+tid_maker()
os.mkdir(depth_path)
os.mkdir(color_path)

def get_image():
    videoTag = False
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    mattag = True
    saved_count = 0
    while True:
        data = pipeline.wait_for_frames()
        depth = data.get_depth_frame()
        color = data.get_color_frame()

        if mattag:
            # 获取内参
            dprofile = depth.get_profile()
            cprofile = color.get_profile()

            cvsprofile = rs.video_stream_profile(cprofile)
            dvsprofile = rs.video_stream_profile(dprofile)

            color_intrin = cvsprofile.get_intrinsics()
            print(color_intrin)
            depth_intrin = dvsprofile.get_intrinsics()
            print(depth_intrin)
            extrin_from_depth2color = dprofile.get_extrinsics_to(dprofile)
            print(extrin_from_depth2color)
            mattag = False
        depth_image = np.asanyarray(depth.get_data())
        color_image = np.asanyarray(color.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        cv2.imshow('2',np.hstack((color_image, depth_colormap)))
        key = cv2.waitKey(30)

        # s 开启视频录像
        if key & 0xFF == ord('s'):
            currentTime = time.time()
            videoTag = True
        if videoTag:
            saved_count += 1
            np.save(os.path.join((depth_path), "{}".format(saved_count)), depth_image)
            cv2.imwrite(os.path.join((color_path), "{}.png".format(saved_count)), color_image)

            time.sleep(0.1)
            if time.time() - currentTime>100:
                #录制100秒退出，可以修改

                exit()

get_image()