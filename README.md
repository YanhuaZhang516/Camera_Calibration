# TASK
- [x] get the extrinsic and intrinsic(fixed) paremeters of the chessboard in Realsense D435i
- [x] get the 3D positon of the corner point in camera coordination (D435i) (two approaches)
- [x] save the data (d435i) in a list
- [ ] Optitrack Detection
  - [x] By ros, get tf between chessboard-world
  - [x] from tf get 3D position
  - [ ] save data in a list
 - [x] create a pyton file, both d435i/Opt in one file
    - [x] create a python file (fucntion: press "c" and it prints "hello")
    - [x] mix with other funciton ("c"-- get data from d435i; "o"-- get data from Optitrack System)
   
# something to check
- [ ] compare the intrinsic parameters from two different approaches
- [ ] check if the point are same in two system
- [ ] check the object coordiantion in chessboard.py

# how to use realsense in ROS
[Realsense2 in ros](https://github.com/YanhuaZhang516/Hiwi_calibration/blob/master/how%20to%20use%20Realsense.md)

# how to use Optitrack System


# Reference
1. [OpenCV Camera Calibration](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html)
2. [OpenCV Camera Calibration and 3D Reconstrunction](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
3. [坐标转换关系](https://www.guyuehome.com/7832)
4. [手眼标定](https://www.guyuehome.com/7871)
5. [pyrealsense2 得到内参系数](https://blog.csdn.net/qq_42393859/article/details/85341279?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.nonecase)
6. [Realsense2相机基本操作指令](https://blog.csdn.net/weixin_42361804/article/details/104144690?biz_id=102&utm_term=realsense2%E6%A0%87%E5%AE%9A&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-104144690&spm=1018.2118.3001.4187)
7. [ros wiki_tf tutorial](http://wiki.ros.org/tf/Tutorials)
8. [tf 教程mooc](https://www.youtube.com/watch?v=ZGMenuXmnrk&list=PLJE-x_JrJ3OJ97FTBIkUhFwdr3bTOVvIB&index=31)
9. [tf easy tutorial](https://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/Lectures/TF%20(transform)%20in%20ROS.pdf)
10. [intelRealsense如何读取彩色图片](https://blog.csdn.net/pursuit_zhangyu/article/details/84258388?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-5.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-5.channel_param)
11. [realsenseD435i reccord rgb images](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py)
12. [tf tutorial 中文](https://www.youtube.com/watch?v=kx0Bwz9dbno)
13. [tf python coding](https://www.youtube.com/watch?v=DLVyc9hOvk8&t=1041s)
