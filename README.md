# Pick and place task with UR5e robot and rg2 gripper

This implements a pick and place task for a UR5e robot and rg2 gripper moving 5 blocks from a table to another one.

## Build && Run

### Using downloaded docker image

```bash
docker pull ghcr.io/mikael-jorda/robotics-test_solution:latest
xhost local:root
docker-compose up
```

### Building docker image locally

Alternatively, you can build the docker image yourself
```bash
sh build_docker_image.sh
xhost local:root
docker-compose -f dpcker-compose-local.yml up
```

### Building from source (not tested)

Or you can build and run without docker images
```bash
sh build_without_docker.sh
cd catkin_ws
catkin build
source devel/setup.bash
roslaunch simple_scene gazebo.launch
```

and in another terminal
```bash
cd catkin_ws
source devel/setup.bash
roslaunch simple_scene simple_planner.launch
```
