## iiwas Cameras

### How to Start the Realsense Camera

First you have to install the realsense ros package

```bash
ias install iiwas_realsense

ias update

ias make
```

After the installation has completed you should be able to start the camera.
```bash
roslaunch realsense2_camera camera_left.launch
```

or

```bash
roslaunch realsense2_camera camera_right.launch
```

or

```bash
roslaunch realsense2_camera camera_both.launch
```

### How to Visualize the Camera Image in Rviz

- Add PointCloud2 to Displays
- As topic choose "/camera/depth_registered/points"
- Choose a Fixed Frame in the Global Options where the tfs of the camera
are attached, e.g. camera_link
- You should see the camera image in rviz

### Camera connection to iiwas
- The cameras are part of the urdf model that can be found in the robots folder of the iiwas_core/iiwas_description package
- All frames of the cameras begin with the robot_name, e.g. L, R for the iiwas arms, as prefix
so they automatically connect to the correct arm.
- The position and rotation of the cameras is also defined in the urdf file

### Launch files
- The used launch files can be found in the launch folder of this repository
    - camera_left.launch
    - camera_right.launch
    - camera_both.launch
- They are build on a modified version of the nodelet.launch.xml that is also provided in the same folder
- The pointclouds get published on a camera specific ros topic, e.g. /camera_left/depth_registered/points for the left camera.
But also on the shared /camera/depth_registered/points topic.

### Possible Problems

- The cameras have to be connected to an USB 3 Slot. You can check if the cameras is on an USB 3 with
  ```bash
  lsusb
  ```
  In the command output the name "Intel Corp." should be registered with BUS02.

- The realsense ros packages need dependencies that have to be installed (are already installed on the lab pc). The dependencies and further installation instructions are explained in the [offical realsense readme](https://git.ias.informatik.tu-darmstadt.de/ias_ros/iiwas_realsense/blob/development/realsense_installation.md)
