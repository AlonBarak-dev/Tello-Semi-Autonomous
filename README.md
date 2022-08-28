# Tello Controller
This repo contains ROS2 package that uses the ROS Joy package  
to control DJI Tello drones with Xbox wireless controller.  
In addition, the drone is able to locate a Face/Person and fly after them.

### How To Build
In the directory of your package, use colcon to build the package:
```ruby
colcon build
```
### Prerequisites
* djitellopy - `pip install djitellopy`

### Details
- The default of the drone is to track Faces, if one wish to detect Person 
  please switch the param -obj to "Person". <br>
- The drone is fully controlled by the user until it detect an Object to track its movement. <br>


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


#### Helped me in the making of this Repository:
- https://github.com/fvilmos/tello_object_tracking <br>
- https://github.com/s4646/Tello-Controller <br>
