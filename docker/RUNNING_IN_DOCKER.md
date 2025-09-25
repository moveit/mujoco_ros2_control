# Running in Docker

We have provided a basic container to run the provided demonstrations in isolation.
The [Dockerfile](Dockerfile) handles installing dependencies, MuJoCo, and building the packages in this environment.

To build, clone the repo and use docker-compose:

```bash
docker-compose build
```

This will create a local image (generally `mujoco_ros2:latest`):

To launch the container use:

```bash
docker-compose up -d mujoco_ros2
docker exec -it mujoco_ros2 bash
...
```

The run script includes the required variables to enable a Ubuntu host to pass through variables to access the display.
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
