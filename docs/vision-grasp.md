# 俯视图像定位与偏移物料抓取

## 当前范围

保留 ROS2 Humble + MoveIt + mock_components，增加独立虚拟相机和 OpenCV 识别节点。
识别节点只处理像素，不读取物料的场景位姿或名义槽位坐标。
虚拟相机从 MoveIt 世界物体读取箱体位置，按针孔投影生成图像。
它没有机器人遮挡、阴影、畸变或物理光照，不是 Gazebo 相机或真实 RGB-D 相机。
支持已知尺寸 40mm、轴对齐、位于料盘平面的橙/黄/紫方块平移。
不支持任意高度、倾斜、旋转抓取或同色多目标，也没有真实夹持力和滑落检测。

## 数据路径

1. `scene_manager` 将初始偏移施加到工件上。偏移只交给场景节点。
2. `virtual_camera` 查询 `/get_planning_scene`，以 5Hz 上限发布
   `/camera/image_raw`（800×800 BGR）和 `/camera/camera_info`。
3. `vision_locator` 只订阅图像，使用 HSV 与连通域提取三个彩色方块。
   利用固定相机标定与工件顶面高度，将中心像素反投影到 `panda_link0`。
4. `/vision/locate_workpieces` 返回同帧的三个中心与原始采集时间。
   `/vision/annotated` 显示检测框、身份和坐标，RViz 增加 Image 显示。
5. `arm_controller` 更新三件工件的碰撞位置，根据所选物料的观测位置计算抓取点。
   到预抓取点后复核，必须有更新图像，任一物料移动超过 3mm 就停止下降。
6. 放回使用本轮观测到的位置，不会重新移动到名义槽位。

固定相机外参位置为 `(0.45,0.25,1.0)`，光轴朝下，图像 x 朝基座 x、图像 y 朝基座 -y。
`fx=fy=900`，主点 `(399.5,399.5)`，工件顶面 z=0.082m，中心 z=0.062m。
单幅图像能得到三维中心，是因为利用了已知平面高度。
真实相机接入需要重新完成内外参标定与畸变校正，不能直接换话题宣称已能准确抓取。

## 间距与拒绝条件

名义中心间距从 8cm 增至 14cm，工件边缘间隙从 4cm 增至 10cm。
料盘 y 尺寸从 0.32m 增至 0.44m。视觉结果须包含三种唯一颜色，轮廓大小与方块相符。
越界、中心距离不足 12cm、缺失、严重遮挡或轮廓异常均拒绝取料。
采集和收到时间均不能过期，三个位姿必须同帧、位于 `panda_link0`。
相机用场景查询发起时间标记采集时间，不把延迟的旧场景标为新图像。
靠近后最多等待 2 秒获取更新帧，避免重复循环中已位于预抓取点时误用同一帧。
视觉失败没有固定坐标后备路径。避碰、完整路径与执行结果检查继续生效。

`slot=0/1/2` 对应橙/黄/紫物料身份，不按最近名义槽位重新分配身份。
料盘占用仍属于程序状态，图像识别不等于真实接触或夹持成功检测。
加工台取回仍使用先前放置的已知位置，本次视觉功能覆盖料盘取料。

## Ubuntu 构建与演示

先退出旧仿真，再更新与编译两个源码包：

```bash
cd ~/demo_ws
git switch debug-pr2-fix
git pull --ff-only origin debug-pr2-fix
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install python3-opencv python3-numpy ros-humble-cv-bridge
colcon build --symlink-install
source install/setup.bash
bash restart_demo.sh repeat_cycle:=false
```

默认平移偏差为 `(12,6)`、`(-9,-7)`、`(6,5)` 毫米，首次启动即可验证不是按名义点取料。
修改 `workpiece_offsets` 可改变三件物料的 dx、dy，单位为米：

```bash
bash restart_demo.sh repeat_cycle:=false workpiece_offsets:="0.02,0.005,-0.015,-0.005,0.01,0.005"
```

每次改变初始条件前先 Ctrl+C 退出旧仿真并等待退出完成。正常完成后可调用 `/task/start` 重跑。
在另一个已加载 ROS 和 install 环境的终端检查图像和定位服务：

```bash
ros2 topic hz /camera/image_raw
ros2 service call /vision/locate_workpieces demo_interfaces/srv/LocateWorkpieces "{}"
ros2 run rqt_image_view rqt_image_view /vision/annotated
```

`rqt_image_view` 可选，若缺少可安装 `ros-humble-rqt-image-view`。RViz 已配置图像显示。
CSV 的 arm feedback 记录两次定位和定位失败原因。
录屏可展示偏移原图、检测框、完整上下料，再补充相机中断后的拒绝取料。
测试中断时可在单轮正常完成后停止 `virtual_camera` 节点，等待超过一秒再发起新任务。

## 离线验证及限制

测试使用真实 OpenCV 执行投影成像、HSV分割、连通域与反投影，覆盖多组平移、
遮挡/同色歧义、过近/越界、过期/错帧、碰撞场景同步、抓取前复核和原位置放回。
像素误差界仅适用于理想虚拟相机，不代表 MoveIt 集成验收或真实抓取精度。
Ubuntu 上仍需验证新槽位可达性、完整避碰轨迹、接口生成和多轮运行。

参考：[OpenCV HSV 分割](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html)、
[ROS cv_bridge](https://index.ros.org/p/cv_bridge/)、
[ROS Image](https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/Image.html)。
