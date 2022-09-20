# TelloAI Competition - Ariel University - K&CG Lab - Starting Kit

## About TelloAI
- The competition is managed by the Computer Science Laboratory at Ariel University - K&CG Lab. <br>
- Number of teams is limited to 20. <br>
- Team size is not limited, each team should have a team leader. <br>
- Each team that passed the the Qualifying stage will receive a Tello drone until the end of the competition. <br>

## Main Events
- TelloAI-101 : Introduction meeting @ 24/10/22 <br>
- TelloAI-102 : Qualification stage @ 31/10/22 <br>
- TelloAI-103 : Final stage @ 7/11/22 <br>

## Prizes:
- Winning team - XXX NIS
- Second team - 50% XXX NIS
- Third team - 25% XXX NIS

## Prerequisites
- Good Knowledge in Python.
- Getting familiar with the Tello SDK.
- Basic understanding of Aruco code.
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
In addition, a code used in Python attached that allows you to <br>
control the drone with the help of your Keyboard while flying <br> 
without interfering with the main algorithm (may save your drone). <br>
In the code we used opencv to detect Aruco code - feel free to use something else. <br>


  ### Python libraries
  * djitellopy - `pip install djitellopy`
  * OpenCV - `pip install opencv-python`
  * NumPy - `pip install numpy`
  * keyboard - `pip install keyboard` - might request sudo permission.
  * threading
  * socket
  * Queue
  * pandas
  * time
  
     
  ## Keyboard Interface
  
  ```ruby
  cd Keyboard-Interface
  ```
  
  ```ruby
  sudo python3 keyboardControl.py
  ```

  ## Link to our YouTube channel
  https://www.youtube.com/watch?v=892dmWhur80&list=PLL4BDIvakL8p3JlQrc3qWykljuYtWlZCS


#### Helped me in the making of this Repository:
- https://github.com/fvilmos/tello_object_tracking <br>
- https://github.com/s4646/Tello-Controller <br>
