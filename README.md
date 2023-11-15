# Services, Logging and Launch Files

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

#### Additional notes and considerations

Screenshots of the `rqt_console` GUI showing log messages are in the
`results/` folder.

The following section assumes that you have an existing ROS2 workspace. If not,
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

## Running the nodes for demonstration

In this exercise, the nodes could either be started individually or together
using a launch file.

### Running the nodes individually (For learning and debugging)

Start the `first_astronaut` node in the first terminal

```bash
ros2 run beginner_tutorials first_astronaut
```

Start the `second_astronaut` node in a new, second terminal

```bash
ros2 run beginner_tutorials second_astronaut
```

>**Note:** You might need to source the package in the new terminal as well
> before running the above command

Hit `Crtl + C` in both the terminals to stop the nodes.

#### Setting the parameters

Parameters should be set **while the nodes are running** from the previous step.

##### First astronaut node

The `first_astronaut` node has a `realization` parameter. It is a string which
conveys the realization the first astronaut could have had...

Start a new terminal and set the parameter this way

```bash
ros2 param set /first_astronaut realization <Your text>
```

##### Second astronaut node

The `second_astronaut` node has a `dramatic_end` parameter. It is a boolean value
with `true` or `false` states. This decides the ending for your meme. Either
just stay cool saying the thing or do the deed and end it in a dramatic fashion!

In the same terminal used for setting the previous node parameter

```bash
ros2 param set /second_astronaut dramatic_end <Your option>
```

### Running the nodes together using launch file (Recommended)

Launch file unifies the above steps into a single line command for convenience.

Both the nodes could be launched together with the desired parameters as arguments,
creating the perfect meme template.

```bash
ros2 launch beginner_tutorials launch.py realization:=<Your text> dramatic_end:=<Your option>
```

To learn more about the launch arguments, use this command

```bash
ros2 launch beginner_tutorials launch.py --show-args
```

>**Note:** Names of the nodes are a bit different when spawned using the launch
> file. Nodes spawned in this way will be under the `/space` namespace.

## Reference

This ROS2 package is a template for the "Always has been" meme.

<p align="center"><img src="https://media.tenor.com/R7iJGnaKOjgAAAAd/lol.gif" height="250"></p>
