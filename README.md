# `tf2`, Unit Testing and Bag Files

ENPM808X - Abhishekh Reddy Munnangi, 119399002

## Get started

### Environment and dependencies

#### Operating system, ROS installation and additional software

- Ubuntu Jammy (22.04)
- ROS2 Humble Hawksbill (Even base installation is sufficient)
- Git

#### Package dependencies

- `rclcpp` - ROS2 C++ Client Library
- `std_msgs` - Standard Messages Library
- `std_srvs` - Standard Services Library
- `ros2launch` - ROS2 Launch Library for Launch file support
- `tf2` - Package API for working with reference frames and transformations
- `tf2_ros` - Command line tools for using the tf2 package
- `ament_cmake_gtest` - GTest for testing the package

#### Additional notes and considerations

- Frames PDF, rosbag outputs are in the results folder, this folder has been
  reorganized in this release

- Screenshots of the `rqt_console` GUI showing log messages are in the
`results/` folder.

- The following section assumes that you have an existing ROS2 workspace. If not,
see [how to create one](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#create-a-new-directory).

### Cloning the repository

The root directory of this repository are contents of a ROS2 package. Hence,
this needs to be cloned to a folder in the `src/` directory of a ROS2 project
workspace.

```console
<Your ROS2 Workspace>/
├── build/
├── install/
├── log/
└── src/
    └── beginner_tutorials/ <-- Repository clones to this folder
        └── <Repository contents>
```

<p align="center">Workspace directory tree with this repository contents in it</p>

Run this command in the `src/` directory of your ROS2 workspace

```bash
git clone https://github.com/armgits/beginner_tutorials.git beginner_tutorials
```

>**Before the next step:** For the first time, ensure that the package
> dependencies are installed. Run these commands from the **root workspace directory**.

```bash
rosdep init && rosdep update
```

```bash
rosdep install --from-paths src -y --ignore-src
```

Build the package in the **root directory** of your ROS2 workspace.

```bash
colcon build --packages-select beginner_tutorials
```

Source the freshly built package

```bash
source install/setup.bash
```

## Testing a node

GTest test case has been added in this homework to test the `first_astronaut` node.

```bash
colcon test --packages-select beginner_tutorials
```

Detailed outputs could be found in the `build/beginner_tutorials/Testing` and
`build/beginner_tutorials/test_results` folders.

## Running the nodes for demonstration

In this exercise, the nodes could either be started individually or together
using a launch file.

### Running the nodes individually

Start the `first_astronaut` node in the first terminal

```bash
ros2 run beginner_tutorials talker
```

Start the `second_astronaut` node in a new, second terminal

```bash
ros2 run beginner_tutorials listener
```

>**Note:** You might need to source the package in the new terminal as well
> before running the above command

Hit `Crtl + C` in both the terminals to stop the nodes.

### Running the nodes together using launch file (Recommended)

Launch file unifies the above steps into a single line command for convenience.

Both the nodes could be launched together with the desired parameters as arguments,
creating the perfect meme template.

```bash
ros2 launch beginner_tutorials launch.py record_bag:=<Your option> realization:=<Your text> dramatic_end:=<Your option>
```

> **Entry format for arguments:**
>
> `record_bag:=` True/False <br>
> `realization:=` "double quotes" <br>
> `dramatic_end:=` true/false

To learn more about the launch arguments, use this command

```bash
ros2 launch beginner_tutorials launch.py --show-args
```

### Recording messages on topics to rosbag

> **Note:** The recorded rosbag will be saved to the **current working directory**.
> i.e. The directory in which the launch file is executed in the terminal.

Use the `record_bag` launch argument in the launch command which accepts a
boolean value of `True` or `False` to record the message activity to a rosbag.

### TF Frames

Two frames `world` and its child frame `talk` are broadcasted using a
`StaticTransformBroadcaster` from the `tf2` package. They can be checked and viewed
while at least the `first_astronaut` node is running.

Read the stamped transform messages

```bash
ros2 run tf2_ros tf2_echo world talk
```

View the frames in a PDF (File is saved in a similar fashion as recording rosbag)

```bash
ros2 run tf2_tools view_frames
```

See the [documentaion](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html#checking-the-frames) for
more information.

## Reference

This ROS2 package is a template for the "Always has been" meme.

<p align="center"><img src="https://media.tenor.com/R7iJGnaKOjgAAAAd/lol.gif" height="250"></p>
