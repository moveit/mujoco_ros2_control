# Running in Docker

We have provided a basic container to run the provided demonstrations in isolation.
The [Dockerfile](Dockerfile) handles installing dependencies, MuJoCo, and building the packages in this environment.

To build, clone the repo and run [the build script](./build.sh), e.g. from the repo root:

```bash
./docker/build.sh
```

This will create a local image (generally `mujoco_ros2:latest`):

```bash
$ docker image ls
REPOSITORY                       TAG       IMAGE ID       CREATED             SIZE
mujoco_ros2                      latest    37874e0d663a   6 minutes ago       1.52GB
```

To launch the container use [the run script](./run.sh), which will drop you into the pre-compiled workspace.
From here launch any of the demo launch scripts:

```bash
$ ./docker/run.sh
root@ubuntu-linux:/home/ros2_ws# ros2 launch mujoco_ros2_control_demos cart_example_position.launch.py
...
```

The run script includes the required variables to enable a Ubuntu host to pass through varibles to access the display.
However, it may be required to give permissions prior to launching the container with:

```bash
# Run on host prior to launching the container
xhost +local:docker
```

To test the MuJoCo install and rendering:

```bash
$ ./docker/run.sh
root@ubuntu-linux:/home/ros2_ws# ${MUJOCO_DIR}/bin/simulate ${MUJOCO_DIR}/model/humanoid/humanoid.xml
```
