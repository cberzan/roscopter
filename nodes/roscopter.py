#!/usr/bin/env python
import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import roscopter.msg
import sys,struct,time,os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))


from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
parser.add_option("--device", dest="device", default="/dev/ttyUSB0", help="serial device")
parser.add_option("--rate", dest="rate", default=10, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-control",dest="enable_control", default=False, help="Enable listning to control messages")

(opts, args) = parser.parse_args()

import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))


#This does not work yet because APM does not have it implemented
#def mav_control(data):
#    '''
#    Set roll, pitch and yaw.
#    roll                      : Desired roll angle in radians (float)
#    pitch                     : Desired pitch angle in radians (float)
#    yaw                       : Desired yaw angle in radians (float)
#    thrust                    : Collective thrust, normalized to 0 .. 1 (float)
#    '''    
#    master.mav.set_roll_pitch_yaw_thrust_send(master.target_system, master.target_component,
#                                                                data.roll, data.pitch, data.yaw, data.thrust)
#
#    print ("sending control: %s"%data)


def send_rc(data):
    master.mav.rc_channels_override_send(master.target_system, master.target_component,data.channel[0],data.channel[1],data.channel[2],data.channel[3],data.channel[4],data.channel[5],data.channel[6],data.channel[7])
    print ("sending rc: %s"%data)


#service callbacks
#def set_mode(mav_mode):
#    master.set_mode_auto()

def set_arm(req):
    master.arducopter_arm()
    return True

def set_disarm(req):
    master.arducopter_disarm()
    return True

pub_gps = rospy.Publisher('gps', NavSatFix)
#pub_imu = rospy.Publisher('imu', Imu)
pub_rc = rospy.Publisher('rc', roscopter.msg.RC)
pub_state = rospy.Publisher('state', roscopter.msg.State)
pub_vfr_hud = rospy.Publisher('vfr_hud', roscopter.msg.VFR_HUD)
pub_attitude = rospy.Publisher('attitude', roscopter.msg.Attitude)
pub_raw_imu =  rospy.Publisher('raw_imu', roscopter.msg.Mavlink_RAW_IMU)
if opts.enable_control:
    #rospy.Subscriber("control", roscopter.msg.Control , mav_control)
    rospy.Subscriber("send_rc", roscopter.msg.RC , send_rc)

#define service callbacks
arm_service = rospy.Service('arm',Empty,set_arm)
disarm_service = rospy.Service('disarm',Empty,set_disarm)


#state
gps_msg = NavSatFix()



def mainloop():
    rospy.init_node('roscopter')
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        msg = master.recv_match(blocking=False)
        if not msg:
            continue
        #print msg.get_type()
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else: 
            msg_type = msg.get_type()
            if msg_type == "RC_CHANNELS_RAW" :
                pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]) 
            if msg_type == "HEARTBEAT":
                pub_state.publish(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 
                                  msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 
                                  mavutil.mode_string_v10(msg))
            if msg_type == "VFR_HUD":
                pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)

            if msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >=3:
                    fix=NavSatStatus.STATUS_FIX
                pub_gps.publish(NavSatFix(latitude = msg.lat/1e07,
                                          longitude = msg.lon/1e07,
                                          altitude = msg.alt/1e03,
                                          status = NavSatStatus(status=fix, service = NavSatStatus.SERVICE_GPS) 
                                          ))
            #pub.publish(String("MSG: %s"%msg))
            if msg_type == "ATTITUDE" :
                pub_attitude.publish(msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)


            if msg_type == "LOCAL_POSITION_NED" :
                print "Local Pos: (%f %f %f) , (%f %f %f)" %(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)

            if msg_type == "RAW_IMU" :
                pub_raw_imu.publish (Header(), msg.time_usec, 
                                     msg.xacc, msg.yacc, msg.zacc, 
                                     msg.xgyro, msg.ygyro, msg.zgyro,
                                     msg.xmag, msg.ymag, msg.zmag)




# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)


# waiting for 10 seconds for the system to be ready
print("Sleeping for 10 seconds to allow system, to be ready")
rospy.sleep(10)
print("Sending all stream request for rate %u" % opts.rate)
#for i in range(0, 3):

master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)

#master.mav.set_mode_send(master.target_system, 
if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException: pass
