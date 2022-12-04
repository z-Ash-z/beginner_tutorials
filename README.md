# Sample Publisher and Subscriber for ROS2
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

- This is a basic package to use and understand the nodes in ROS2.
- This package uses one node to publish string messages and another node to subscribe to those string messages.

## Dependencies

- Ubuntu 20.04 LTS or 22.04 LTS running `ROS2 Humble`.
- `colcon`
- `rosdep`

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

- Install the necessary dependencies before proceeding.
```
cd <ros2 workspace folder>
rosdep install -i --from-path src --rosdistro humble -y
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
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command.
```
ros2 run pub_sub publisher
```
- To modify the parameters while launching the publisher, add the parameter arguments as follows:
```
ros2 run pub_sub publisher --ros-args -p <parameter_name>:=<parameter_value>
```
- There are three `<parameter_name>`'s offered with this package.
    1. topic_name (type = String). Usage `topic_name:='Chatter`. This changes the topic to which this Node publishes.
    2. publish_message (type = String). Usage `publish_message:='Hello`. This changes the message that is published by this node.
    3. publish_interval (type = int). Usage `publish_interval:=10000`. This changes the rate (in ms) at which the message is published by this node.

## Running the subscriber
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command.
```
ros2 run pub_sub subscriber
```

## Running the service
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the publisher is running.
```
ros2 service call /change_publisher_string pub_sub/srv/StringChange "{new_string: '<string of choice>'}"
```
- After entering the string of choice switch to the publisher or the subscriber tab to see the change.

## Visualizing the nodes
- The nodes that are generated are as follows:
![graph](/pub_sub/results/screenshots/rosgraph.png)

## Running launch files
- Before running the launch files close all terminals.
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command.
```
ros2 launch pub_sub <launch_file>.py 
```
- There are four `<launch_file>`s offered with this package.
    1. `_pub_sub_default_launch.py` - To launch the publisher and subscriber simultaneously using the default values.
    2.  `_pub_sub_custom_launch.py <arg_name>:=<parameter_value>` - To launch the publisher and subscriber simultaneously using the custom values for the publisher.
        - There are three parameters that can be changed from command line:
            1. topic_name (type = String). Usage `topic_name:='Chatter`. This changes the topic to which this Node publishes.
            2. publish_message (type = String). Usage `publish_message:='Hello`. This changes the message that is published by this node.
            3. publish_interval (type = int). Usage `publish_interval:=10000`. This changes the rate (in ms) at which the message is published by this node.
    3. `_pub_sub_error_launch.py` - To launch the publisher and subscriber simultaneously with the publishing interval set to `500ms` therefore raising a `ERROR` level severity.
    4. `_pub_sub_fatal_launch.py` - To launch the publisher and subscriber simultaneously with the publishing interval set to `400ms` therefore raising a `FATAL` level severity. This also stops the publisher.
    > **_NOTE:_** The publisher exits when it reaches `FATAL` level issue but the subscriber continues to run.

### Checking the logger level
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the publisher or any of the launch files are running.
```
ros2 run rqt_console rqt_console
```
- Sample screensnip of the `rqt_console`.
![rqt_console](/pub_sub/results/screenshots/rqt_console.png)

## Checking the published frames
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the publisher or any of the launch files are running.
    1. For viewing the frames being broadcasted use:
        ```
        ros2 run tf2_tools view_frames
        ```
        - The result while running the publisher is here: [diagram](/pub_sub/results/frames_2022-12-01_01.21.06.pdf)

    2. For seeing the report between two frames use:
        ```
        ros2 run tf2_ros tf2_echo talk world
        ```
