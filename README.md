## Docker ROS Kinetic & Gazebo9 with X11 support

Based on
* [docker_ros_kinetic_cuda9](https://github.com/gandrein/docker_ros_kinetic_cuda9)



### Gazebo-9
ROS Kinetic comes with Gazebo-7 pre-installed. This image will remove Gazebo-7 and install Gazebo-9 following the steps provided in this [Gazebo tutorial](http://gazebosim.org/tutorials/?tut=ros_wrapper_versions).



### Running a container

Run with 
```
docker run --runtime=NVIDIA -p 7000:7000 -p 8181:8181 <image-name>
```
The container shares the X11 unix socket with the host and its network interface on port 7000.
The Cloud9-IDE will be deployed on Port 8181


### Testing

#### Test ROS

While inside the container call `roscore` in Cloud9 terminal. The `ros master` should be launched. 




#### Test Gazebo
If the _glxgears_ GUI opened without errors, you will be able to run Gazebo Client. Open in browser at first port 7000 to generate a display. In terminal on Cloud9 (Port 8181) type in
```
gazebo --verbose
```
The Gazebo GUI in the environment on port 7000 should open. Note that when launching the container with the `--verbose` flag you will see 
```
[Wrn] [ModelDatabase.cc:340] Getting models from[http://gazebosim.org/models/]. This may take a few seconds.
```
This implies that Gazebo is trying to connect to the online model database to grab the available models. This will take some time, and the Gazebo splash may freeze on your screen. 






