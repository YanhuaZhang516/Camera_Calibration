# How to use optitrack system 
1. 右选所有tracker并构建刚体（rigid-body）可获取中心点的坐标
2. 如何连接两台电脑：

step1: 开启主电脑

```
~$ ias_ros
~$ roslaunch darias_launch darias_full_set.launch
```
step2: 开启副电脑

```
~$ init_ros
~$ rostopic list
~$ rviz
```


# 参考资料
1. [tf](https://github.com/YanhuaZhang516/Linux_ROS/blob/master/tf_ros.md)
