# TelloAI Competition - Ariel University - K&CG Lab - Starting Kit

## About TelloAI
- The competition is managed by the Computer Science Laboratory at Ariel University - K&CG Lab. <br>
- Number of teams is limited to 20. <br>
- Team size is not limited, each team should have a team leader. <br>
- Each team that passed the the Qualifying stage will receive a Tello drone until the end of the competition. <br>

## Main Events
- TelloAI-101 : Introduction meeting @ 24/10/22 <br>
- TelloAI-102 : Qualification stage @ 31/10/22 <br>
- TelloAI-103 : Semi-Finals stage @ ?/11/22 <br>
- TelloAI-104 : Final stage @ 7/11/22 <br>

## Prizes:
- Winning team - XXX NIS
- Second team - 50% XXX NIS
- Third team - 25% XXX NIS

## Prerequisites
- Good Knowledge in Python.
- Getting familiar with the Tello SDK.
- Basic understanding of Aruco code.
- Basic knowledge in ROS2 (OPT)
- A winning team!

## Links to expand knowledge on the required subjects
- https://www.youtube.com/watch?v=7UzPIpXn-g4
- https://www.youtube.com/watch?v=k4Q03_WrpwM
- https://github.com/fvilmos/tello_object_tracking
- https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf -- Tello SDK
- https://github.com/tariq86/tello_edu.py
- https://www.youtube.com/watch?v=lqBRYkWSmjI
- https://github.com/waseemtannous/Autonomous-Drone-Scanning-and-Mapping
- https://www.aeroroboticscomp.com/fall2022
- https://www.arxiv-vanity.com/papers/2104.09815/
- https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/


## Starting-Kit
This repo contains code that knows how to communicate with the Tello drone, <br>
Receive information and video streams from him in real time. <br>
In addition, a code used in ROS2 is attached that allows you to <br>
control the drone with the help of a wireless Xbox remote while flying <br> 
without interfering with the main algorithm (may save your drone). <br>
In the code we used opencv to detect Aruco code - feel free to use something else. <br>


## How to use the Repo
  - Note: I've wrote the code using ROS2, meaning that I've used additional software other than Python, 
          it is optinal to use ROS2 (Xbox controller).
          Therefore, one can read the code itself (ROS2 is a small part) and gain understaning on the Tello API from it. <br>
          
  ### Prerequisites
  * djitellopy - `pip install djitellopy`
  * OpenCV - `pip install opencv-python`
  * NumPy - `pip install numpy`
  * ROS2 foxy
  * colcon
  
  ## ROS2 Interface
  
  ```ruby
  cd ROS2-Interface
  ```
  
  ### Build
  In the directory of your package, use colcon to build the package:
  ```ruby
  colcon build
  ```
  ### How To Use
  1. Open a terminal, run the joy node:
  ```ruby
  ros2 run joy joy_node
  ```
  2. In a new terminal, move to the package's built directory
  ```ruby
  cd <directory_of_package>
  ```
  3. Then source the setup script:
  ```ruby
  source install/setup.bash
  ```
  4. Run the package
  ```ruby
  ros2 run tello_controller object_track_xbox
  ```
     
  ## Python only (Keyboard) Interface
  
  ```ruby
  cd Keyboard-Interface
  ```
  
  ```ruby
  sudo python3 keyboardControl.py
  ```

#### Helped me in the making of this Repository:
- https://github.com/fvilmos/tello_object_tracking <br>
- https://github.com/s4646/Tello-Controller <br>
