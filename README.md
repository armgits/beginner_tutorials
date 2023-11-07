# First Publisher/Subscriber

ENPM808X - Abhishekh Reddy Munnangi, 119399002

## Get started

### Environment and dependencies

#### Operating system, ROS installation and additional software

- Ubuntu Jammy (22.04)
- ROS2 Humble (Even base installation is sufficient)
- git

#### Package dependencies

- `rclcpp` - ROS2 C++ Client Library
- `std_msgs` - Standard Messages Library

Assuming that you have an existing ROS2 workspace for the next step. If not, see [how to create one](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#create-a-new-directory).

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
git clone -b ros_pub_sub https://github.com/armgits/beginner_tutorials.git beginner_tutorials
```

Navigate back to the root directory of your ROS2 workspace and build the package

```bash
colcon build --packages-select beginner_tutorials
```

Source the freshly built package

```bash
source install/setup.bash
```

## Running the nodes for demonstration

Start the `talker` node in the first terminal

```bash
ros2 run beginner_tutorials talker
```

Start the `listener` node in a new, second terminal

```bash
ros2 run beginner_tutorials listener
```

Hit `Crtl + C` in both the terminals to stop the nodes.
