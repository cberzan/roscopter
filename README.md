This is a cleaned-up version of [roscopter][1].

Tested with ROS groovy.

Docs in progress.


## Building roscopter

Create a ROS / Catkin workspace as shown [here][2] in `~/catkin_ws` and
activate it (`source devel/setup.bash`).

```
# Install dependencies:
sudo aptitude install ros-groovy-sensor-msgs

# Clone the roscopter repo and submodules (mavlink):
cd ~/catkin_ws/src
git clone https://github.com/cberzan/roscopter.git
git submodule update --init

# Check that ROS knows about roscopter:
rospack list |grep roscopter

# Build this sucker:
rosmake roscopter
```


## Building mavlink

We need to generate the python bindings for mavlink. There should be a way to
generate this more easily, but I can't figure out where it's documented. I
tried "cd build; cmake ..; make", but that fails with an error (`No such file
or directory:
'.../roscopter/mavlink/pymavlink/generator/C/include_v0.9/mavlink_conversions.h'`)

I did it like this:

```
cd roscopter/mavlink
./pymavlink/generator/mavgen.py \
    --output pymavlink/dialects/v10/ardupilotmega.py \
    message_definitions/v1.0/ardupilotmega.xml
```


[1]: https://github.com/cberzan/roscopter.git
[2]: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace
