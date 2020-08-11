# calibration
此处相机标定主要分为两部分：
1. 完成 Realsense D435i的内参获取以及求出外参数，求出真实世界坐标与相机坐标的关系
2. 通过 Optitrack System 获得中心点的实际坐标，求出其与D435i 的转换坐标
3. 匹配 D435i 和 Optitrack System 两个相机系统

# 任务
- [x] get the extrinsic and intrinsic(fixed) paremeters of the chessboard in Realsense D435i
- [x] get the 3D positon of the corner point in camera coordination (D435i) (two approaches)
- [ ] save the data (d435i) in a list
- [ ] Optitrack Detection
  - [ ] By ros, get tf between chessboard-world
  - [ ] from tf get 3D position
  - [ ] save data in a list
 - [ ] create a pyton file, both d435i/Opt in one file
    - [ ] create a python file (fucntion: press "c" and it prints "hello")
    - [ ] mix with other funciton ("c"-- get data from d435i; "o"-- get data from Optitrack System)
   
# 如何使用 realsense
[Realsense2 在 ros中的使用](https://github.com/YanhuaZhang516/Hiwi_calibration/blob/master/how%20to%20use%20Realsense.md)

# 如何使用 Optitrack System


# 参考资料
1. [OpenCV Camera Calibration](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html)
2. [OpenCV Camera Calibration and 3D Reconstrunction](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
3. [坐标转换关系](https://www.guyuehome.com/7832)
4. [手眼标定](https://www.guyuehome.com/7871)
5. [pyrealsense2 得到内参系数](https://blog.csdn.net/qq_42393859/article/details/85341279?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.nonecase)
6. [Realsense2相机基本操作指令](https://blog.csdn.net/weixin_42361804/article/details/104144690?biz_id=102&utm_term=realsense2%E6%A0%87%E5%AE%9A&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-104144690&spm=1018.2118.3001.4187)
