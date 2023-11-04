# First Publisher/Subscriber

ENPM808X - Abhishekh Reddy Munnangi, 119399002

## Get started

The root directory of this repository are contents of a ROS2 package. Hence,
this needs to be cloned to a folder in the `src/` directory of a ROS2 project
workspace.

```console
<your_ros2_workspace>/
├── build/
├── install/
├── log/
└── src/
    └── beginner_tutorials/ <-- Repository clones to this folder
        └── <Repository contents>
```

<p align="center">Workspace directory tree with this repository contents in it</p>

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

Switch to the `ros_pub_sub` branch of this repository

```bash
git checkout ros_pub_sub
```
