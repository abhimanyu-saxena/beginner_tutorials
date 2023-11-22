# Basic Pub - Sub using C++

Abhimanyu Saxena (asaxena4@umd.edu)

## Get started

### Environment and dependencies

#### System Dependencies

- Ubuntu 22.04
- ROS2 Humble

#### Package dependencies

- `rclcpp` - ROS2 C++ Client Library
- `std_msgs` - Standard Messages Library

The next steps assume you have set up a ROS2 workspace. 

### Cloning the repository

Run this command in the `src/` directory of your ROS2 workspace

```bash
git clone https://github.com/abhimanyu-saxena/beginner_tutorials.git beginner_tutorials
```

Build the package using

```bash
colcon build --packages-select beginner_tutorials
```

Source package

```bash
source install/setup.bash
```

The `result` folder contains cpplint and cppceck outputs along with screenshots of the results.

## Running the nodes for demonstration

Start the `talker` node in the first terminal

```bash
ros2 run beginner_tutorials talker
```

Start the `listener` node in the second terminal

```bash
ros2 run beginner_tutorials listener
```

>**Note:** Source the workspace in the new terminal as well

Start the `service` node in the third terminal

```bash
ros2 run beginner_tutorials server
```

>**Note:** Source the workspace in the new terminal as well

Change the `frequency` node in the fourth terminal to `0.0` to see FATAL log info

```bash
ros2 param set /minimal_publisher freq 0.0
```

>**Note:** Source the workspace in the new terminal as well

Hit `Crtl + C` in both the terminals to stop the nodes.

This will look something like this:

![](results/info_logging_terminal.png)

The node graph looks something like this:

![](results/node_graph.png)