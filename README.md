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


## Testing roscopter

When connecting our APM via USB, it is seen as `/dev/ttyACM0`. Had to add
myself to the `dialout` group to get permissions to read / write this device.

Start roscore in one terminal.

Run roscopter in another terminal:

```
cd ~/catkin_ws/src/roscopter
nodes/roscopter.py --device=/dev/ttyACM0 --baudrate=115200
```

You should see that it receives a heartbeat from the APM, and then displays the
APM initialization messages, ending with "Ready to FLY".

In a third terminal, you can do for example `rostopic echo /attitude` to watch
the attitude topic.

See [original docs][1] for more details. Note that the original docs are wrong:
you have to use `--baudrate` instead of `--rate`. See the source for the
meaning of these parameters.



[1]: https://code.google.com/p/roscopter/
[2]: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace
