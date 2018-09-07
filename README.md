# gazebo_ros_3dx
This is a ROS package containing a URDF model for the Pioneer 3DX, which uses the Hector sensors package to publish sonar data and
also exports a camera.

It also subscribes to two topics, leftmotor and rightmotor (handled by remapping in world.launch, so to get these to work you'll need
to launch with that file or modify that file) (or work out how to do it yourself!)



