# gazebo_ros_3dx
This is a ROS package containing a URDF model for the Pioneer 3DX, which uses the Hector sensors package to publish sonar data and
also exports a camera.

It also subscribes to two topics, leftmotor and rightmotor (handled by remapping in world.launch, so to get these to work you'll need
to launch with that file or modify that file) (or work out how to do it yourself!)

To use the thing
* put it all into your Catkin workspace as a package and make the workspace.
* remove the last node from `launch/sim.launch` (the "diamondpublish" node)
* remove the "lightsensor" node from `defs/jcf.xacro` (which uses my lightsensor library to publish a light sensor based on simulated camera data)
* start roscore
* run `roslaunch gazebo_ros_3dx sim.launch gui:=true` to start Gazebo, set up the world, add the robot and start the Gazebo GUI.



## What's where

* `defs/jcf.xacro` contains the definition of the robot using meshes from the meshes directory.
* `defs/sonar_sensor.urdf.xacro` contains a definition of a sonar sensor macro, using the Hector sensor packages.
* `defs/sonar_pos.urdf.xacro` contains invocations of the above macro to position the sonars
on the robot.
* `defs/sonarpos.ang` is an Angort program to generate the sonar position Xacro file above.
* `config/p3dx_control.yaml` contains parameter definitions for the simulated motor con-
trollers.
* `launch/sim.launch` will launch Gazebo and a simulated robot with all sensors and actuators
connected to ROS. It uses world.launch to invoke Gazebo and load the appropriate world.
*It will also start diamondpublish.py. You probably won't want that, so remove it.*
* `scripts/diamondpublish.py` will read the robot location from Gazebo and republish it to
Diamond Apparatus (my comms system) - *ignore it*.
* `world.launch` is invoked from sim.launch and starts Gazebo with a world. It also remaps
the default hardware controller command nodes to leftmotor and rightmotor.
* `worlds/foo.sdf` is the default world loaded, generated by...
* `sdfgen.ang` is a handy Angort library for building worlds.

The lightsensor gazebo package contains the light sensor simulator, and also the LightSensor
message used by both the real and simulated sensor.

## Dependencies
The system absolutely needs

* `hector_sensors_gazebo` - a ROS package that sets up simulated sensors for Gazebo

to make the sensors work. It also uses the following packages of my own, which you don't need:

* https://github.com/jimfinnis/DiamondApparatus (handles non-ROS comms)
* https://github.com/jimfinnis/lightsensor_gazebo (generates a simulated omnidirectional light sensor)
* https://github.com/jimfinnis/angort (my weird programming language that's used to generate the world `.sdf` file)
