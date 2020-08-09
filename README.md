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
   


# 参考资料
1. [OpenCV Camera Calibration](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html)
2. [OpenCV Camera Calibration and 3D Reconstrunction](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
3. [坐标转换关系](https://www.guyuehome.com/7832)
4. [手眼标定](https://www.guyuehome.com/7871)
