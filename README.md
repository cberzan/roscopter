roscopter
=========
This is a cleaned-up version of [roscopter][1]. Tested with ROS Groovy and Hydro.

Building
--------
Assuming you have a Catkin workspace set up in `~/catkin_ws` as described [here][2]:
```bash
# Install dependencies (replace hydro by groovy if necessary):
sudo aptitude install ros-hydro-sensor-msgs python-serial python-tz

# Clone this repository and initialise the `mavlink` submodule:
cd ~/catkin_ws/src
git clone https://github.com/ssk2/roscopter.git
cd roscopter
git submodule update --init

# Build:
cd ~/catkin_ws/
catkin_make --pkg roscopter

# Generate mavlink python bindings:
cd ~/catkin_ws/src/roscopter/mavlink
./pymavlink/generator/mavgen.py --output pymavlink/dialects/v10/ardupilotmega.py message_definitions/v1.0/ardupilotmega.xml
```

Testing
-------
When connecting our APM via USB, it is seen as `/dev/ttyACM0`. 

You may need to add the user running roscopter to the `dialout` group (`sudo usermod -a -G dialout username`) to get permission to read / write to this device.

Start roscore in one terminal.
```
roscore
```

Run roscopter in another terminal:

```
rosrun roscopter roscopter_node.py  --device=/dev/ttyACM0 --baudrate=115200
```

You should see that it receives a heartbeat from the APM, and then displays the
APM initialization messages, ending with "Ready to FLY".

In a third terminal, you can do for example `rostopic echo /attitude` to watch
the attitude topic.

See [original docs][1] for more details. (Note that the original docs are wrong:
you have to use `--baudrate` instead of `--rate`. See the source for the
meaning of these parameters.)

Performance
-----------
pymavlink continually polls the serial port to look for new messages - which is processor intensive. 

Setting `timeout=1` in `roscopter/mavlink/pymavlink/mavutil.py` as below proves to help the CPU usage significantly:
```
class mavserial(mavfile):
    '''a serial mavlink port'''
    def __init__(self, device, baud=115200, autoreconnect=False, source_system=255):
        import serial
        self.baud = baud
        self.device = device
        self.autoreconnect = autoreconnect
        self.port = serial.Serial(self.device, self.baud, timeout=1,
                                  dsrdtr=False, rtscts=False, xonxoff=False)
```


[1]: https://code.google.com/p/roscopter/
[2]: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace
