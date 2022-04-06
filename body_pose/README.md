[body_pose](https://github.com/BluewhaleRobot/body_pose) 是一个人体姿态识别的软件包。这个软件包可以从图片中识别出人体的耳朵，眼睛，鼻子，四肢的共17个特征点。其实现是通过深度学习网络利用tensorflow框架。此软件还支持多人同时识别。

![0_1530878202843_6f71b239-27ae-453a-94d9-94de429c8c84-image.png](https://community.bwbot.org/assets/uploads/files/1530878219879-6f71b239-27ae-453a-94d9-94de429c8c84-image-resized.png) 

### 安装
1. 安装tensorflow
由于小强上面没有Nvidia显卡,所以我们安装CPU版本的Tensorflow。小强的CPU支持一些高效的指令集，默认的Tensorflow为了能够在更多平台上运行，没有使用这些指令集。性能实际上没有发挥到最高。我们可以安装开启了这些优化指令的版本。

```bash
sudo pip2 install --ignore-installed --upgrade "https://github.com/lakshayg/tensorflow-build/releases/download/tf1.12.0-macOS-mojave-ubuntu16.04-py2-py3/tensorflow-1.12.0-cp27-cp27mu-linux_x86_64.whl"
```

2. 安装软件包相关依赖

```bash
sudo pip install scipy scikit-image matplotlib pyyaml easydict cython munkres==1.0.12
```

3. 下载相关模型

```
git clone https://github.com/BluewhaleRobot/body_pose
# 单人模型
cd src/models/mpii
./download_models.sh
$ cd -
# 多人模型

./compile.sh
cd models/coco
./download_models.sh
cd -
```

### 运行

1. 运行单人识别测试程序

```bash
# 在src文件夹内运行
TF_CUDNN_USE_AUTOTUNE=0 python demo/singleperson.py
```

运行成功后可以看到下图

![0_1530879542859_94996fd8-2ca8-418f-ac9f-11da6221380a-image.png](https://community.bwbot.org/assets/uploads/files/1530879564082-94996fd8-2ca8-418f-ac9f-11da6221380a-image-resized.png) 

2. 运行多人识别的例子

```bash
TF_CUDNN_USE_AUTOTUNE=0 python demo/demo_multiperson.py
```

成功运行后显示下面的图像

![0_1530879857466_158a04a4-defc-498a-87e3-fdae8ee9ab41-image.png](https://community.bwbot.org/assets/uploads/files/1530879880168-158a04a4-defc-498a-87e3-fdae8ee9ab41-image-resized.png) 

3. 运行ROS服务

```bash
roslaunch body_pose body_pose_test.launch
```
这个launch文件在小强上可以直接运行。但是如果不是在小强上你需要把usb_cam的节点换成自己的摄像头节点。
服务启动成功后会自动的识别摄像头中的图像。处理的结果会显示在 `/body_pose_tester/processed_image` topic 里面。可以利用摄像头显示处理结果。

```
rosrun image_view image_view image:=/body_pose_tester/processed_image
```

可以看到图中标了一些圆圈。这些都是被识别出的人体特征位置。

 ![0_1530880373290_QQ图片20180706203240.jpg](https://community.bwbot.org/assets/uploads/files/1530880392561-qq图片20180706203240-resized.jpg) 

### 在自己的程序中使用此服务

可以在自己的launch文件中添加这样的内容来启动节点

```xml
<node name="body_pose" pkg="body_pose" type="body_pose_node.py">
    <param name="process_width" value="200" />
    <param name="process_height" value="150" />
    <param name="conf_min_count" value="3" />
</node>
```

参数说明
process_width 内部处理宽度，process_height内部处理高度。可能采集的图片很大，这样处理起来非常费时间。可以通过这两个参数对图片进行压缩以提高处理速度。
conf_min_count最小可接受点数。如果在一个人上识别出的特征点小于这个值，则认为这个识别对象并不是人。

发布的服务
服务默认发布在 `~get_body_pose`可以类型为[BodyPose.srv](https://github.com/BluewhaleRobot/body_pose/blob/master/srv/BodyPose.srv)。

详细的使用方法可以参照 [body_pose_test_node.py ](https://github.com/BluewhaleRobot/body_pose/blob/master/src/body_pose_test_node.py)文件。
