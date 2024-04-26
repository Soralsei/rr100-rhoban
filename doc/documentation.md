# RR100 Rhoban documentation
<!-- <style>
  .figure {
    margin: 12pt auto;
    width: 100%;
    display: flex;
    flex-direction : column;
    justify-content: center;
    align-items: center;
    row-gap: 8pt;
  }
  .figure > p,i {
    font-size: 12pt;
  }
</style> -->

## Package and tools documentation
- `docker` : https://docs.docker.com/
- `slam_toolbox` : https://github.com/SteveMacenski/slam_toolbox/blob/noetic-devel/README.md#Introduction
- `pointcloud_to_laserscan` : http://wiki.ros.org/pointcloud_to_laserscan
- `move_base` : http://wiki.ros.org/move_base
  - `costmap_2d` : http://wiki.ros.org/costmap_2d
    - `costmap_2d::StaticLayer` : http://wiki.ros.org/costmap_2d/hydro/staticmap
    - `costmap_2d::VoxelLayer` : http://wiki.ros.org/costmap_2d/hydro/obstacles#VoxelCostmapPlugin
    - `costmap_2d::InflationLayer` : http://wiki.ros.org/costmap_2d/hydro/inflation
  - `global_planner` : http://wiki.ros.org/global_planner
  - `teb_local_planner` : http://wiki.ros.org/teb_local_planner
- `robot_localization` : 
  - http://docs.ros.org/en/noetic/api/robot_localization/html/index.html
  - https://github.com/methylDragon/ros-sensor-fusion-tutorial/tree/master


## Docker basics
This section is useful for those who either don't have access to a PC running Ubuntu 18.04 or want to avoid installing every dependency of this project to their computer by keeping it in a standalone container.

### Architecture basics
Docker is based on a client-server architecture with the client talking to a local or remote daemon (`dockerd`) which handles building images, running and distributing containers. These images and containers are called *Docker objects*. 

<div>
  <img src="resources/docker-architecture.webp"/>
  <p align="center"><i>Docker's general architecture (Source: <a href="https://docs.docker.com/get-started/overview/#docker-architecture">Docker's documentation</a>)</i></p>
</div>

#### Images
An image is a read-only template with instructions for creating a Docker container. Often, an image is based on another image, with some additional customization. For example, you may build an image which is based on the ubuntu image, but installs the Apache web server and your application, as well as the configuration details needed to make your application run.

To create these images, Docker uses its own syntax in files (usually) named `Dockerfile`. Almost every instruction in a Dockerfile creates a new *layer* in the image. When you change the Dockerfile and rebuild the image, *only* the layers that have changed are rebuilt. This includes the layers directly changed  (example: a modified file which is copied to the image with a `COPY` instruction) and every subsequent layer after that one in the same *stage* (which we will describe later).

#### Containers
A container is a runnable instance of an image. You can create, start, stop, move, or delete a container using the Docker API or CLI. You can connect a container to one or more networks, attach storage to it, or even create a new image based on its current state.

By default, a container is relatively well isolated from other containers and its host machine. You can control how isolated a container's network, storage, or other underlying subsystems are from other containers or from the host machine.

A container is defined by its image as well as any configuration options you provide to it when you create or start it. When a container is removed, any changes to its state that aren't stored in persistent storage disappear.

### Using Docker
To build you own image and run containers, you will have to write a Dockerfile and use Docker's command line interface (CLI). This documentation won't guide you on how to go about building your own images but a detailed guide is a available on the online [Docker documentation](https://docs.docker.com/build/guide/). We will however give you the most useful Dockerfile instructions and important Docker CLI commands to know.

#### Dockerfile instructions
One of the first (and usually *the* first) instruction you will encounter in Dockerfiles is the `FROM` instruction. This tells the builder which parent image to inherit from when building your own image. This means we can use pre-existing images (such a the `ros:$ROS_DISTRO` images) to build our applications without having to install create the environment from scratch.

For example, this instructions sets your base image to the 18.04  release of Ubuntu:
```Dockerfile
FROM ubuntu:18.04
```
All subsequent instructions are then executed inside of this base Ubuntu image.
> [!NOTE]
> The notation ubuntu:18.04, follows the name:tag standard for naming Docker images which you will also see in this project's Dockerfile

Another common set of instructions is the `ADD`/`COPY` instructions. These instruction are very similar in that they both copy files into a container but `ADD` has additional functionnality. `ADD` can automatically extract standard compression formats into a container and can also be used to fetch resources from a URL.
> [!TIP]
> You'll mostly want to use `COPY` for copying files from one stage to another in a multi-stage build or if you need files from your build context (your host project files) inside of your containers. `ADD` is best for when you need to download a remote artifact as part of your build and should be used instead of manually fetching files with `wget` or `curl` and `tar`. See the [Best practices for Dockerfile instructions](https://docs.docker.com/develop/develop-images/instructions/#add-or-copy) for more details

Another pair of useful and similar instructions is the `ARG`/`ENV` pair of instructions. `ARG` and `ENV` both declare and assign variables. `ARG` defines a build-time variable that can be used to declare build arguments for the Dockerfile as well as variables that can only be used in the *build* context (meaning that they're out-of-scope when using `RUN` instructions which are executed inside intermediate containers during the build). `ENV` is used to declare and assign **environment variables** in containers and can be used in `RUN` instructions.

Finally, one of the most useful (if not *the* most useful) instructions in Dockerfiles is the `RUN` instruction. It is used to execute arbitrary shell commands to create a new layer on top of the current image. The `RUN` instruction is commonly used along `apt`/`apt-get` to install required packages inside images.

> [!WARNING]
> Because every layer in an image increases its final size on disk, it is advised to use as little `RUN` instructions as possible by combining them (with the `&&` shell operator) in a single command. Also the `RUN apt-get` command has several counter-intuitive behaviors to look out for. See [here](https://docs.docker.com/develop/develop-images/instructions/#run) for details

#### Docker CLI
The Docker CLI is used for building images, running containers and managing your Docker local and remote registries (where your docker images are stored). 

##### Building
To build your docker images, you will need to know the `docker build` command. This command takes various arguments with the most commonly used ones being `--tag <name>(?:\:<tag>)?`, `--target <Stage in Dockerfile>` and `--build-arg <ARG_NAME>=<VALUE>`, followed by the path the build context (which should contain your Dockerfile). This will start the building process of your Docker image described by the Dockerfile passed to the command. For a complete overview of this command , you can check out the [official reference](https://docs.docker.com/reference/cli/docker/image/build/) for it.

Example command:
```console
user@machine:~/hello_world$ docker build --tag hello-world:v1.0.0 .
```

##### Running containers
To run containers, you will need to use the `docker run` command. The `docker run` command runs a command in a new container, pulling the image if needed and starting the container. The command takes the following format `docker run [OPTIONS] <IMAGE> [COMMAND] [ARG...]` and if a command is not specified, Docker only executes the image's `ENTRYPOINT` or `CMD`.
> [!NOTE]
> See [here](https://docs.docker.com/reference/dockerfile/#understand-how-cmd-and-entrypoint-interact) for details about `ENTRYPOINT` and `CMD`. Something that isn't mentionned in the linked section is that docker images inherit `ENTRYPOINT` and `CMD` from its base image.

`docker run` can take many options, with the most common ones being:
- `-i, --interactive` which keeps the containers `STDIN` open to inputs
- `-t, --tty` which creates a pseudo-terminal and attaches it to the container
- `--mount <key>=<value>` which allows you to mount volumes, host-directories, and tmpfs mounts in a container
- `-e, --env <NAME>=<VALUE>` which is used to set environment variables inside of the container.
- `--name NAME` which gives a name to the container that can be used to manage said container.
- `--net, --network=<network>` to connect the container to a specific network (or `host` to use the host's network)
- `-p, --publish (?:<host ip>\:)?<host port>:(?:<container ip>\:)?<container port>` which binds the container's port to the hosts's port on a specific network interface (if specified) or all interfaces.
- `--expose <port>` which exposes the specified container's port without publishing it to the host system's interfaces

Example command:
```console
user@machine:~/hello_world$ docker run -ti --name hello-container bash
```

## ROS basics
> [!NOTE]
> Under construction

## Dockerfile structure
The Dockerfile for this project is structured in a multi-stage manner and inherits from the official Docker `ros:noetic` image. There are in total 3 building stages and 2 target stages in our building process. To optimize build times, we define 2 independent stages `apt-depends` and `cacher` that can be run in parallel by Docker's builder. The former stage's purpose is to download and install dependencies and useful packages for this project. This stage is later used as a base for our `builder` stage that will handleactually building this project.

The latter stage (`cacher`) is used to cache `package.xml` files from our project's packages which declare these packages dependencies as they're less likely to change during development :
```Dockerfile
WORKDIR $WORKSPACE/src
COPY src .

# Separate package.xml files in /tmp directory
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find . -name "package.xml" | \
    xargs cp --parents -t /tmp/opt && \
    find . -name "CATKIN_IGNORE" | \
    xargs cp --parents -t /tmp/opt || true
```
We copy the source directory to this stage and copy every `package.xml`/`CATKIN_IGNORE` file to a temporary directory while keeping the original source tree structure. In this same stage, we separate dependencies from simulator packages and from actual project packages to be able to compile them in separate docker layers later.

Next comes the `builder` stage that inherits from the `apt-depends` stage. In `builder` we first copy the `package.xml` files from the `cacher` stage into our final workspace and run `rosdep install` to install our packages' dependencies. Then, we copy our separated packages and compile them in their own layers to make use of Docker's caching capabilities.

Finally we have the two target stages, `simulation` and `real`. In `simulation`, we copy the simulation packages into our workspace and compile them separately whereas in `real`, we skip these packages as they are not needed and set our `ROS_MASTER_URI` to the RR100 robot and our `ROS_IP` to our IP address.

## Build and run script
> [!NOTE]
> Under construction

## Overview of the project
### Package interaction
This project makes use of many packages oriented towards autonomous robot navigation, all of which are standard in the ROS ecosystem. These packages all handle a different aspect of navigation, namely mapping, localizing, and path planning for the robot and we will go over how each of them works in a later section.

For now, let's focus on how each of these packages are integrated in our autonomous navigation package and how they intreact with each other. Below, a diagram illustrates how each package interacts with the robot and the other packages.
<div>
  <img src="resources/rr100_package_diagram.png"/><br>
  <p align="center"><i>RR100 navigation package diagram</i></p>
</div>

First, `rslidar_laserscan` (internally uses `pointcloud_to_laserscan`) takes in point clouds recorded by the RSLiDAR-16 of the robot and converts them to a planar 2D point cloud in the standard ROS format `sensor_msgs/LaserScan`. This converted point cloud is then piped to the SLAM package `slam_toolbox`, which reads the robot's transform tree (more specifically the transform from the base frame to the LiDAR frame and from the odometry frame to the base frame) and the 2D laserscan to compute an occupancy grid used as a map (how this is done will be detailed in a later section) which is then published.

In parallel, `robot_localization` (a pose estimation package built with Kalman filters) takes in wheel encoder odometry data and IMU data published by the robot and fuses these sensor measurements to estimate to robot's pose in the odometry frame and publish this pose to the `tf` tree (which is used by `slam_toolbox`).

Finally `move_base`, which can be subdivided into 3 packages (2 path planning packages and a costmap computing package), uses sensor data, a SLAM or a static map, and the robot's pose estimation to :
- compute costmaps (global and local) in a *grid* format which are used by both planners to plan a path towards a goal pose 
- plan a global path towards the goal pose by using the global costmap as a graph and finding the least costly path (in terms of distances and cost on the costmap) from the starting pose node to the goal pose node
- plan a local path to follow the global path (which includes unplanned obstacle avoidance) using the local costmap in a similar fashion to the global planner
- compute velocity commands to follow the local path

### Node and topic interaction
> [!NOTE]
> Under construction
<div class="figure" >
  <img src="resources/nav_node_graph_no_tf.png"/>
  <p align="center"><i>Package node graph (without tf connections), ellipses represent nodes and rectangles represent topics</i></p>
</div>

## Per-package description
> [!NOTE]
> Under construction
### rslidar_laserscan
### rr100_slam
### rr100_localization
### rr100_navigation
