# 2022嵌入式芯片与系统设计竞赛_龙芯赛道

## 基于龙芯教育派与双目立体相机的3D扫描

首先启动TB6600.py，转台进行转动。
然后执行get_mat_save_pic.py文件，获取识别的三维模型的点云信息，进入UI后，按s健，进行点云信息的获取，保存的RGB图在color中存储，保存的点云生成.npy保存在depth中。


Depth文件夹下应该存放.npy文件下。

![](media/fb2d5f1a8d16983b9aadc81aa65efe2d.png)

首先执行from_depth2cloud，将深度图转为点云，存放在pcd文件夹下

接着执行deal_cloud，deal_cloud使用两个参数，硬性删除背景和地平面，处理完后的点云存放在hard_value_pcd文件夹下

接着执行peizhun，将处理后的点云配准，点云匹配使用ICP算法，首先将每二十帧点云配准，生成的中间文件存放在dealing_pcd中，之后将这些配准后的文件再次配准，生成fusioned_pcd.pcd

接着执行get_mesh，将融合后的点云制作成网格模型，保存为mesh.ply

在执行上述.py文件时，都要把注释掉的执行命令反注释。

参数设置：

![](media/730c725e2078917881d4ebdca78db5ba.png)

Hard_value_z与hard_value_y，该参数是硬性删除高于z与高于y的点云（ransac平面去除）
可以在vision中使用第一帧点云进行调试

![](media/f97c83e6519cdaeb4292b07fd836744e.png)

后面两个max为最大变换阈值，第一个函数的参数应小一点0.3，0.03，第二个应大一点50，10

使用ui界面时会很卡，因为在深度图转点云，点云融合，点云生成网格时耗时都特别长，耗时预计如下：

深度图转点云：

50min

点云匹配：

30min

生成模型：

5min
