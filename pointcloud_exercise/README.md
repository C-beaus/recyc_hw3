# Pointcloud Exercise

This package demonstrates point cloud processing in a Gazebo simulation environment. It includes a camera sensor and an object. The package shows how to process the point cloud to extract and visualize flat patches of the object in the form of point cloud data.


## Installation

Follow these steps to set up the package:

1. **Download the package**  
   - Download the `pointcloud.zip` file.
   - Extract it to your ROS 2 workspace's `src` folder.

2. **Build the package**  
   - Open a terminal and navigate to your workspace directory:
     ```bash
     colcon build
     ```

3. **Source the workspace**  
   - After building, source the setup script:
     ```bash
     . install/setup.bash
     ```

4. **Launch the simulation**  
   - To start the simulation, run:
     ```bash
     ros2 launch pointcloud_exercise simulation.launch.py
     ```

    - You should be able to see the camera and arm in Gazebo. The red indicator represents the camera's view direction.
    - To adjust the camera's spawn location, modify the parameters of the **spawn_entity** node in the [simulation.launch.py](/pointcloud_exercise/launch/simulation.launch.py) file. 

5. **Run the point cloud transformation**  
   - To run the point cloud transformation node for extracting the flat patch of an objct:
     ```bash
     ros2 run pointcloud_exercise transform_pointcloud
     ```
     - Point Cloud is loaded from .npy file. (Line 27)
     - Point Cloud should be in 3xN format where N are the number of points. 

6. **Visualize in RViz**  
   - Launch RViz2 to visualize the data:
     ```bash
     ros2 run rviz2 rviz2
     ```

    - In RViz2, follow these steps
        - Change the Fixed Frame from "map" to "world" under the Global Options.
        - Under Add by Topic, select "/realsense/points/PointCloud2" to visualize the camera view of the object.
        - Select "/transformed_pointcloud_data/PointCloud2" to visualize flat patch of the object.
        - Enable the Invert z-axis option in the Views window on the right.

## Spawning Objects
- You can spawn various objects in Gazebo from the hidden folder **~/.gazebo/models**. To access this folder:
    - Enable Show Hidden Files in your file manager (usually found in the top-right corner).
    - You can also modify the [simulation.world](/pointcloud_exercise/worlds/simulation.world) file to add or remove objects in the Gazebo simulation.

## PCL Tutorials
You will be required to use PCL Library. Please refer to the following links for tutorials.
```
https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
```

## Few Commands for PCL 
```
sudo apt-get update
```
For installing packages related to the Point Cloud Library (PCL) for ROS Humble.
```
sudo apt-get install ros-humble-pcl-*
```

For installing the Point Cloud Library (PCL) development files and libraries.
```
sudo apt install libpcl-dev
```
