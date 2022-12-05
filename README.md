# Sample Publisher and Subscriber for ROS2
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

- This is a basic package to use and understand the nodes in ROS2.
- This package uses one node to publish string messages and another node to subscribe to those string messages.

## Dependencies

- Ubuntu 20.04 LTS or 22.04 LTS running `ROS2 Humble`.
- `colcon`
- `rosdep`  

# Using the package

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

## Running a service
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the publisher is running.
```
ros2 service call /change_publisher_string pub_sub/srv/StringChange "{new_string: '<string of choice>'}"
```
- After entering the string of choice switch to the publisher or the subscriber tab to see the change.

## Running launch files
- Before running the launch files close all terminals.
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command.
```
ros2 launch pub_sub <launch_file>.py 
```
- There are four `<launch_file>`s offered with this package.
    1. `_pub_sub_default_launch.py` - To launch the publisher and subscriber simultaneously using the default values.
    2.  `_pub_sub_custom_launch.py <arg_name>:=<parameter_value>` - To launch the publisher and subscriber simultaneously using the custom values for the publisher.
        - There are four parameters that can be changed from command line:
            1. topic_name (type = String). Usage `topic_name:='Chatter`. This changes the topic to which this Node publishes.
            2. publish_message (type = String). Usage `publish_message:='Hello`. This changes the message that is published by this node.
            3. publish_interval (type = int). Usage `publish_interval:=10000`. This changes the rate (in ms) at which the message is published by this node.
            4. record_bag (type = bool). Usage `record_bag:=True`. This helps record the ros topics into a ros bag.
            > To view all the available options run: ```ros2 launch pub_sub _pub_sub_custom_launch.py -s``` 
    3. `_pub_sub_error_launch.py` - To launch the publisher and subscriber simultaneously with the publishing interval set to `500ms` therefore raising a `ERROR` level severity.
    4. `_pub_sub_fatal_launch.py` - To launch the publisher and subscriber simultaneously with the publishing interval set to `400ms` therefore raising a `FATAL` level severity. This also stops the publisher.
    > **_NOTE:_** The publisher exits when it reaches `FATAL` level issue but the subscriber continues to run.

# Testing the package

## Testing using colcon
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the publisher or any of the launch files are running.
```
colcon test --event-handlers console_direct+ --packages-select pub_sub
```
> The client services are tested and you can see the results in the terminal, with details about the tests that passed.

# Results

## Recording a ros bad
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the publisher or any of the launch files are running.
```
ros2 bag record --all -o package_output
```
- A sample of the results are stored in the [results/record_all](/pub_sub/results/record_all) folder. 

## Visualizing the nodes
- Close all terminals and run a launch file.
- The nodes that are generated are as follows:
![graph](/pub_sub/results/screenshots/rosgraph.png)

## Checking the logger level
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the publisher or any of the launch files are running.
```
ros2 run rqt_console rqt_console
```
- Sample screensnip of the `rqt_console`.
![rqt_console](/pub_sub/results/screenshots/rqt_console.png)

## Running CPP lint
- For style guide analysis, from the root directory of the project, run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/name,-build/include_order,-runtime/explicit --recursive pub_sub/. > pub_sub/results/cpplint.txt
```
- The results are in [cpplint.txt](/pub_sub/results/cpplint.txt)

## Running CPP check
- For static code analysis, from the root directory of the project run:
```
cppcheck --enable=all --std=c++17 pub_sub/app/ pub_sub/include/pub_sub/ pub_sub/src --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > pub_sub/results/cppcheck.txt
```
- The results are in [cppcheck.txt](/pub_sub/results/cppcheck.txt)

## Checking the published frames
- In a new terminal (`Ctrl+Shift+T`) source both humble and your package. Then type the following command while the publisher or any of the launch files are running.
    1. For viewing the frames being broadcasted use:
        ```
        ros2 run tf2_tools view_frames
        ```
        - The result while running the publisher is here: [diagram](/pub_sub/results/frames_2022-12-05_16.45.07.pdf)

    2. For seeing the report between two frames use:
        ```
        ros2 run tf2_ros tf2_echo talk world
        ```