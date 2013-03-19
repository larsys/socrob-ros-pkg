
NODE_NAME = 'raposang_gui'
import roslib; roslib.load_manifest(NODE_NAME)
import rospy

from sensor_msgs.msg import *
from raposang_msgs.msg import *

WLAN = "wlan0"
RATE = 5

class Telemetry:
    data = {}

    def handler_imu(self, msg):
        self.data["orientation"] = (msg.header.stamp, msg.orientation)

    def handler_odometry(self, msg):
        self.data["arm_angle"] = (msg.header.stamp, msg.arm_angle)

    def main(self):
        rospy.Subscriber("/imu/data", Imu, self.handler_imu, queue_size=1)
        rospy.Subscriber("/raposang/odometry", RaposaOdometry, self.handler_odometry, queue_size=1)
        self.pub = rospy.Publisher("/raposang/telemetry", RaposaTelemetry)
        rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)
        rate = rospy.Rate(RATE)
        rospy.loginfo("Streaming telemetry")
        while not rospy.is_shutdown():
            self.update_wireless()
            msg = RaposaTelemetry()
            msg.header.stamp = rospy.get_rostime()
            for (k,v) in self.data.items():
                setattr(msg, k, v[1])
            self.pub.publish(msg)
            rate.sleep()

    def update_wireless(self):
        LABELS = ("wlan_link", "wlan_level", "wlan_noise")
        with open("/proc/net/wireless") as fh:
            stamp = rospy.get_rostime()
            for ln in fh:
                fs = ln.strip().split()
                if fs[0]=="%s:"%(WLAN):
                    for (i,l) in enumerate(LABELS):
                        self.data[l] = (stamp, int( fs[2+i].strip(".") ))
                    break



if __name__=="__main__":
    Telemetry().main()

# EOF
