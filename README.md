
# Butler Robot 

## Requirements
 - ROS 2 Humble
 - Gazebo Fortress
 - [Navigation 2](https://navigation.ros.org/build_instructions/index.html#install)
   
## Requirements for setting up gazebo robot simulation package
link to original package : [Github Repo](https://github.com/art-e-fact/navigation2_ignition_gazebo_example)

### inside navigation2_ignition_gazebo_example

setup and build
```

# Install Nav2 dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Import source dependencies
pip3 install vcstool
vcs import --input deps.repos src

# Install rosrep dependencies
rosdep install -y -r -i  --from-paths . 

```

## Usage

run gazebo simulation and robot manager node :

```
ros2 launch butler_bot butler_bot.launch.py
```

run the service node that starts takes input from user in terminal to emulate confirmations

```
ros2 run butler_bot conf_server 
```

send order to robot manager node by calling the `action`

```
ros2 action send_goal /take_order butler_interfaces/action/TableOrder '{order: ["table1", "table2", "table3"]}' --feedback
```

To cancel orders call service with table name

```
ros2 service call /cancel_order butler_interfaces/srv/CancelOrder '{tableid: "table3"}'
```

## Demo Video

[![Delivery Cancel Demo](https://img.youtube.com/vi/OeQ2RmKH4vA/0.jpg)](https://youtu.be/OeQ2RmKH4vA)


Simple delivery video 


[![Demo](https://img.youtube.com/vi/WktzcJttmdk/0.jpg)](https://youtu.be/WktzcJttmdk)

Delivery cancel video 
