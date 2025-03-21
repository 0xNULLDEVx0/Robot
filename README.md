# Robot

This application was built with ROS2 jazzy, with dependencies found in a basic ROS2 install.

To get this working, ensure ROS2 is installed then go into the root directory learnRos. Reason for the name is due to my own new discoveries when working with jazzy. Can never stop learning right?

Before anything, you must source the files. run **source install/setup.bash**

Run the command **colcon build** in the root directory (/learnRos).

The robot is a 6 DOF robot with links,joints, and limits. The limits are there to prevent the robot from colliding with itself and performing unnatural transformations.

There are two ways to run this, as I was unable to get the GUI and the subscriber listening to each other.

# Test the robot with the GUI RViz

To test this with the GUI Joint State publisher, go into the build directory where the launch file is located (install/six_axis_pubsub/share/six_axis_pubsub/robot_launch.py)
Replace the value of the **controller_config** and the **urdf_file_path** to the location on your system. They will be in the src folder of /learnRos/src/six_axis_pubsub/src. In order for the config to be run on a Windows machine (as I am using linux) I left this as an input variable since linux and windows use two different filepath naming conventions. **Copy the full path**
E.g (for my system) /home/user/Documents/Code/learnRos/src/six_axis_pubsub/src/controller_config.yaml | /home/user/Documents/Code/learnRos/src/six_axis_pubsub/src/robot_arm.urdf


When these paths are updated, run colcon build oncemore. Then launch it using the command **ros2 launch six_axis_pubsub robot_launch.py**. You will get an empty view, as well as some sliders to control the joints. The view will be empty, so add the robot by going to Add -> RobotMode. Set the topic description to /robot_description. You should now see the robot arm with multi-colored cylinders. You can control each joint by adjusting the sliders, to test the joint limitations. You can also randomize it if curious.
![Screenshot from 2025-03-21 11-56-43](https://github.com/user-attachments/assets/89c279b4-5d30-4185-81f0-03351d61b1fb)

# Test the robot with key commands

To test this without the GUI, you need two terminal windows. Source both of them, then run **colcon build**. 
In terminal window one, start the robot arm with the command **ros2 run six_axis_pubsub arm_controller**
In the second terminal window, start the teleop node with the command **ros2 run six_axis_pubsub teleop_node**.

You will have the choice to manipulate each joint individually. Select the joint first, then press **w** to raise the joint, and **s** to lower the joint. When a joint limit is reached, that joint can no longer be moved. When an input is sent, the arm_controller will respond with a confirmation.
To change joints, press **q** to quit modifying the current joint, then press a number 1-6 to modify the joint of choice.
![image](https://github.com/user-attachments/assets/8fa71fbe-6c40-4b10-b8c4-9a8a3aac4c26)
![image](https://github.com/user-attachments/assets/87657e0e-3b49-45c3-ac04-53f482ef9e19)
![image](https://github.com/user-attachments/assets/f0e8ff4e-51e7-41ac-bef8-c14a4c20509e)
![image](https://github.com/user-attachments/assets/54262465-6ebc-4a9e-8a40-6cfb19baea36)

I tried getting this to work hand in hand with the GUI, but was having some issues. So I decided to do both! Both are taking the principles of sending commands to the joints, and limitations, except one is by keyboard and one is by a GUI controller.

# Troubleshooting

## Can't find packages

This solution uses native ROS2 packages, listed below in the CMAKE file. If a package is missing, ensure the jazzy install is working (or Humble)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

## GUI interface is not showing the joint control sliders

Ensure that the URDF filepath is correct in the robot_launch.py. If this launch file somehow gets deleted when building, the backup file is located in the src folder called robot_launch_back.py. Insert it in the original spot in the install folder.

