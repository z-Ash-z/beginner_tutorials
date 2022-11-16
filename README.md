# Sample Publisher and Subscriber for ROS2

- This is a basic package to use and understand the nodes in ROS2.
- This package uses one node to publish string messages and another node to subscribe to those string messages.

## Installing dependencies

- To install the necessary dependencies before proceeding.
```
cd <ros2 workspace folder>
rosdep install -i --from-path src --rosdistro humble -y
```

## Building the package

- source the ROS Humble based on installation.
```
. /opt/ros/humble/setup.bash
OR
. ~/ros2_humble/ros2-linux/setup.bash
```

- Clone the repository
```
cd <ros2 workspace folder>/src
git clone https://github.com/z-Ash-z/beginner_tutorials.git
cd ..
```

- Use colcon to build the package.
```
colcon build --packages-select pub_sub
```

- Source the package based on installation type.
```
. install/setup.bash
    OR
. install/local_setup.bash
```

## Running the publisher
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. The type the following command.
```
ros2 run pub_sub publisher
```
## Running the subscriber
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. The type the following command.
```
ros2 run pub_sub subscriber
```