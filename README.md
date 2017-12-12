## Docker ROS Kinetic & Gazebo8 with OpenGL support

Use the Dockerfile and the [./build.sh](./build.sh) script to create a Docker image containing ROS Kinetic (full version) and Gazebo-8 for Ubuntu Xenial. The image will have a shared X11 and support for OpenGL and Nvidia hardware acceleration. 

Based on
* [docker_ros_kinetic_cuda9](https://github.com/gandrein/docker_ros_kinetic_cuda9)

### Requirements

This docker image has been build and tested on a machine running Ubuntu 16.04 with the following packages installed
* `docker` version `17.09.0-ce`
* [`nvidia-docker`](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-1.0)) version `1.0`


### Gazebo-8
ROS Kinetic comes with Gazebo-7 pre-installed. This image will remove Gazebo-7 and install Gazebo-8 following the steps provided in this [Gazebo tutorial](http://gazebosim.org/tutorials/?tut=ros_wrapper_versions).

### Nvidia Hardware acceleration

For details on how to install the `nvidia-docker1`, follow the steps provided at the official [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-1.0)) repository.

**Note** at the date of writting this, `nvidia-docker2` did not have OpenGL support as mentioned in [issue #534](https://github.com/NVIDIA/nvidia-docker/issues/534). To be able to open GUIs, such as the Gazebo Client GUI, use `nvidia-docker1`.

### Building the image

Run [./build.sh](./build.sh). This will create the image with a name given as first argument (`IMAGE_NAME`) or when missing, it will be named for you as `ros_kinetic_gazebo8`.

```
./build.sh IMAGE_NAME
```

### Running a container

Run [./run.sh](./run.sh) with the default or chosen name from the step above. This will run and remove the docker image upon exit (i.e., it is ran with the `--rm` flag). The container shares the X11 unix socket with the host and its network interface.

The script checks for the version of `nvidia-docker` and calls `docker run` differently based on found version. If no `nvidia-docker` is found, a normal container will be spawned

```
./run.sh IMAGE_NAME
```

### Testing

#### Test ROS

While inside the container call `roscore`. The `ros master` should be launched. 

#### Test Nvidia-Docker

To test the Nvidia HW acceleration inside the ROS-Kinetic container started with [./run.sh](./run.sh), call `nvidia-smi` from a container's terminal
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 387.12                 Driver Version: 387.12                    |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  Quadro M2000M       Off  | 00000000:01:00.0  On |                  N/A |
| N/A   43C    P5    N/A /  N/A |    792MiB /  4042MiB |     34%      Default |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
+-----------------------------------------------------------------------------+
```

Alternatively, in a terminal, you can directly run
```
nvidia-docker run --rm nvidia/cuda nvidia-smi
```
as described [here](https://github.com/NVIDIA/nvidia-docker/wiki/Usage) and [here](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-1.0)).

#### OpenGL support
From a container's terminal run `glxgears`. The _glxgears_ GUI should open.  If you encounter 
```
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
X Error of failed request:  BadValue (integer parameter out of range for operation)
  Major opcode of failed request:  154 (GLX)
  Minor opcode of failed request:  3 (X_GLXCreateContext)
  Value in failed request:  0x0
  Serial number of failed request:  35
  Current serial number in output stream:  37
```
then `nvidia-docker1` is not working properly. 

#### Test Gazebo
If the _glxgears_ GUI opened without errors, you will be able to run Gazebo Client. Start a container of this image with [./run.sh](./run.sh). In the terminal type
```
gazebo --verbose
```
The Gazebo GUI should open. Note that when launching the container with the `--verbose` flag you will see 
```
[Wrn] [ModelDatabase.cc:340] Getting models from[http://gazebosim.org/models/]. This may take a few seconds.
```
This implies that Gazebo is trying to connect to the online model database to grab the available models. This will take some time, and the Gazebo splash may freeze on your screen. 

#### Test Gazebo-ROS integration
Start two Docker containers of this image with [./run.sh](./run.sh). 
* in the first Docker container's terminal start the `roscore` master from the terminal.
* in the second Docker container's terminal start a ROS node that launches Gazebo
```
rosrun gazebo_ros gazebo --verbose
```




