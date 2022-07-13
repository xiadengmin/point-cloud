#coding=gbk
from PyQt5 import QtCore,QtGui,QtWidgets
import pyautogui as pag
from PyQt5.Qt import *
import sys
import cv2
import numpy as np
import datetime
import os
import time
import vision
import from_depth2cloud
import peizhun
import get_mesh


class data(QtWidgets.QMainWindow):
    def __init__(self):
        super(data, self).__init__()
        self.init_ui()
        self.init_slot()

    def init_ui(self):
        self.setFixedSize(360,360)
        self.main_window = QtWidgets.QWidget()
        self.main_layout = QtWidgets.QGridLayout()
        self.main_window.setLayout(self.main_layout)
        self.setCentralWidget(self.main_window)
        self.show_depth_image = QtWidgets.QPushButton("显示深度图像")
        self.main_layout.addWidget(self.show_depth_image)
        self.depth2point = QtWidgets.QPushButton("深度图像转点云")
        self.main_layout.addWidget(self.depth2point)
        self.show_point_image = QtWidgets.QPushButton("显示点云")
        self.main_layout.addWidget(self.show_point_image)
        self.show_hard_value_point_image = QtWidgets.QPushButton("显示处理后的点云")
        self.main_layout.addWidget(self.show_hard_value_point_image)
        self.show_fusion_point_image = QtWidgets.QPushButton("显示融合后的点云")
        self.main_layout.addWidget(self.show_fusion_point_image)
        self.point_fusion = QtWidgets.QPushButton("点云匹配")
        self.main_layout.addWidget(self.point_fusion)
        self.get_amesh = QtWidgets.QPushButton("生成模型")
        self.main_layout.addWidget(self.get_amesh)
        self.show_mesh = QtWidgets.QPushButton("显示网格")
        self.main_layout.addWidget(self.show_mesh)

        self.show_label = QtWidgets.QLabel("选择文件:")
        self.main_layout.addWidget(self.show_label)
        self.get_file = QtWidgets.QLineEdit()
        self.main_layout.addWidget(self.get_file)
        self.spacerItem = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum,
                                                QtWidgets.QSizePolicy.Expanding)
        self.main_layout.addItem(self.spacerItem)


        # self.show_label = QtWidgets.QLabel("输出信息:")
        # self.main_layout.addWidget(self.show_label)
        # self.show_msg = QtWidgets.QTextEdit()
        # self.main_layout.addWidget(self.show_msg)

    def init_slot(self):
        self.show_depth_image.clicked.connect(self.Show_depth_image)
        self.depth2point.clicked.connect(self.Depth2point)
        self.show_point_image.clicked.connect(self.Show_point_image)
        self.point_fusion.clicked.connect(self.Point_fusion)
        self.get_amesh.clicked.connect(self.Get_mesh)
        self.show_mesh.clicked.connect(self.Show_mesh)
        self.show_hard_value_point_image.clicked.connect(self.Show_hard_value_point_image)
        self.show_fusion_point_image.clicked.connect(self.Show_fusion_point_image)

    def Show_depth_image(self):
        text = self.get_file.text()
        print(text)
        if len(text)==0:
            vision.show_depth()
        else:
            vision.show_depth(text)
        print("Show_depth_image")

    def Depth2point(self):
        """Runs the main function."""
        from_depth2cloud.get_all_pcd()
        print("Depth2point")

    def Show_point_image(self):
        text = self.get_file.text()
        print(text)
        if len(text)==0:
            vision.show_pcd()
        else:
            vision.show_pcd(text)
        print("Show_point_image")

    def Show_hard_value_point_image(self):
        text = self.get_file.text()
        print(text)
        if len(text)==0:
            vision.show_hard_value_pcd()
        else:
            vision.show_hard_value_pcd(text)
        print("Show_hard_value_point_image")

    def Show_fusion_point_image(self):
        text = self.get_file.text()
        print(text)
        if len(text)==0:
            vision.show_fusion_point_image()
        else:
            vision.show_fusion_point_image(text)
        print("Show_fusioned_point_image")


    def Point_fusion(self):
        peizhun.get_combined_pcd()
        print("Point_fusion")

    def Get_mesh(self):
        get_mesh.mesh()
        print("Get_mesh")

    def Show_mesh(self):
        text = self.get_file.text()
        print(text)
        if len(text)==0:
            vision.show_mesh()
        else:
            vision.show_mesh(text)
        print("Show_mesh")


def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = data()
    gui.show()
    sys.exit(app.exec_())
if __name__ == '__main__':
    main()
