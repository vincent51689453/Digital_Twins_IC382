# Gazebo_IC382_Simulation
This platform provides a simulation environment for Hong Kong Polytechnic University Industrial Centre Course (IC382) Rescue Robot Simulation . The simulation tasks are mainly focus on computer vision and deep learning. The robot should have the ability to control itself in order to reach the goal meanwhile, it should be able to classify the turning directions based on the traffic light pattern.


**1. Demo and Layout**
---------------------------
### 1.1 Simulation Field
![image](https://github.com/vincent51689453/Gazebo_IC382_Simulation/blob/master/git_image/demo_layout.png)

### 1.2 Target Performance
![image](https://github.com/vincent51689453/Gazebo_IC382_Simulation/blob/master/git_image/demo_gazebo_sim.gif)

**2. Virtual Vision Integration**
---------------------------
### 2.1 Basic virtual camera 
![image](https://github.com/vincent51689453/Gazebo_IC382_Simulation/blob/master/git_image/opencv_camera.png)

### 2.2 Lane Detection
![image](https://github.com/vincent51689453/Gazebo_IC382_Simulation/blob/master/git_image/cv2_lane_detetor.png)



**3. Deep Learning Integration**
---------------------------
### 3.1 Vision based control
![image](https://github.com/vincent51689453/Digital_Twins_IC382/blob/master/git_image/gazebo-ai-demo.gif)

### 3.2 Tensorboard Training Evaluation
The network trained based on ssdmobilienet_v1 with coco dataset.
![image](https://github.com/vincent51689453/Digital_Twins_IC382/blob/master/git_image/learning_rate_less.JPG)

![image](https://github.com/vincent51689453/Digital_Twins_IC382/blob/master/git_image/loss_smoothed.JPG)

![image](https://github.com/vincent51689453/Digital_Twins_IC382/blob/master/git_image/loss_smoothed_less.JPG)




**4. Graph and Visualization**
---------------------------
### 4.1 Tracked robot URDF
![image](https://github.com/vincent51689453/Gazebo_IC382_Simulation/blob/master/git_image/robot_urdf.png)

### 4.2 ROS graph
![image](https://github.com/vincent51689453/Gazebo_IC382_Simulation/blob/master/git_image/rqt_graph.png)


**5. Help **
---------------------------
### 5.1 Usage of scripts
1. camera_checker.sh: it is a script for you to check whether the python can receive gazebo virutal camera image or not. It will display the raw image from ros topic

2. cv_lane_detector.sh: it is a script to perform lane detection using the virutal camera as a source but it is not compatible for tensorflow (py3)

3. dataset_prepare.sh: it is a script that keep on taking saving pictures in order to help you prepare object detection dataset.

4. fake_velocity.sh: it is a script to publish twist message to control the robot directly.

5. run_gazebo.sh: it is a script to start gazebo and load the world

6. simple_controller.sh: it is a script to allow manual control of the robot.

7. visualize_nodes.sh: it is a script to generate ros graph

8. camera_publish.sh: it is a script to publish the image as a jpg from python2 opencv

9. camera_subscriber.sh: it is a script to show published image and perform lane detection (testing)

10. TensorRT-ROS-Bridge.sh: it is a script which does not belong to ROS. It loads published image and perform AI object detection and other image processsing.

11. vision_control.sh: it is a script to perform PID control based on vision and deep learning.

### 5.2 Startup Procedures
1. ./run_gazebo.sh (load the world) [ROS]
2. ./camera_publish.sh (publish the image) [ROS]
3. ./TensorRT-ROS-Bridge.sh (enable AI and vision processing module) [NOT ROS]
4. ./vision_control.sh [ROS]

**Remeber to put correct models to ~/.gazebo/models, otherwise the world cannnot be loaded. Also, put budiling_editor_models under ~/

