
import sys
import math
import pickle
import os
import re
import fcntl
import subprocess
import numpy as np
import numpy.linalg as la
from Tkinter import *
from tkFont import *

NODE_NAME = 'raposang_gui'
import roslib; roslib.load_manifest(NODE_NAME)
import rospy

from sensor_msgs.msg import *
from raposang_msgs.msg import *
from geometry_msgs.msg import *

ROBOT_HOSTNAME = "raposang"

LABELS = ["heading", ("body pitch", "frontal angle"), "body roll"]
MARGIN = 20
RADIUS = 50

VALUE_FONT_SCALE = 1.5

ARROW = np.array([(-.8,-.1), (-.8,.1), (.25,.1),  (.25,.2),   (.8,0),   (.25,-.2), (.25,-.1)])

BACK  = np.array([(-.8,-.3), (-.8,.3), (-.6,.3),  (-.6,.2),  (.6,.2), (.6,.3),   (.8,.3),
                  (.8,-.3),  (.6,-.3), (.6,-.2), (-.6,-.2), (-.6,-.3)])

SIDE  = np.array([[ -5.00000000e-01,   2.00000000e-01],
                  [ -5.76536686e-01,   1.84775907e-01],
                  [ -6.41421356e-01,   1.41421356e-01],
                  [ -6.84775907e-01,   7.65366865e-02],
                  [ -7.00000000e-01,   2.44929360e-17],
                  [ -6.84775907e-01,  -7.65366865e-02],
                  [ -6.41421356e-01,  -1.41421356e-01],
                  [ -5.76536686e-01,  -1.84775907e-01],
                  [ -5.00000000e-01,  -2.00000000e-01],
                  [ -3.67394040e-17,  -2.00000000e-01],
                  [  7.65366865e-02,  -1.84775907e-01],
                  [  1.41421356e-01,  -1.41421356e-01],
                  [  1.84775907e-01,  -7.65366865e-02],
                  [  2.00000000e-01,   0.00000000e+00],
                  [  1.84775907e-01,   7.65366865e-02],
                  [  1.41421356e-01,   1.41421356e-01],
                  [  7.65366865e-02,   1.84775907e-01],
                  [  1.22464680e-17,   2.00000000e-01]])

ARM   = np.array([[  1.00000000e-01,  -2.00000000e-01],
                  [  1.76536686e-01,  -1.84775907e-01],
                  [  2.41421356e-01,  -1.41421356e-01],
                  [  2.84775907e-01,  -7.65366865e-02],
                  [  3.00000000e-01,   0.00000000e+00],
                  [  2.84775907e-01,   7.65366865e-02],
                  [  2.41421356e-01,   1.41421356e-01],
                  [  1.76536686e-01,   1.84775907e-01],
                  [  1.00000000e-01,   2.00000000e-01],
                  [  5.00000000e-01,   2.00000000e-01],
                  [  5.76536686e-01,   1.84775907e-01],
                  [  6.41421356e-01,   1.41421356e-01],
                  [  6.84775907e-01,   7.65366865e-02],
                  [  7.00000000e-01,   0.00000000e+00],
                  [  6.84775907e-01,  -7.65366865e-02],
                  [  6.41421356e-01,  -1.41421356e-01],
                  [  5.76536686e-01,  -1.84775907e-01],
                  [  5.00000000e-01,  -2.00000000e-01]])




def RollPitchYaw(R):
    beta = math.atan2(-R[2,0], math.sqrt(R[0,0]**2 + R[1,0]**2))
    cb = math.cos(beta)
    alpha = math.atan2(R[1,0]/cb, R[0,0]/cb)
    gamma = math.atan2(R[2,1]/cb, R[2,2]/cb)
    return np.array([gamma, beta, alpha])

class Viewer(Frame):
    data = dict(arm_angle=0, orientation=Quaternion(x=0, y=0, z=0, w=1))

    def __init__(self):
        self.master = Tk()
        self.LABEL_FONT = Font()
        self.VALUE_FONT = Font(size=int(VALUE_FONT_SCALE*Font().actual("size")))
        Frame.__init__(self, self.master)

    def main(self, use_telemetry=False):
        self.pack()
        # Configure master
        self.master.title("IMU Monitor -- %s mode"%("TELEMETRY" if use_telemetry else "LOCAL"))
        self.master.protocol('WM_DELETE_WINDOW', self.quit)
        self.master.resizable(0, 0)
        # Create widgets
        self.canvas = Canvas(self, width=len(LABELS)*(4*RADIUS+MARGIN)+MARGIN, height=2*RADIUS+2*MARGIN)
        self.canvas.pack(side=TOP, fill=BOTH, expand=1)
        self.wlan_label_var = StringVar()
        Label(self, textvariable=self.wlan_label_var, font=self.VALUE_FONT, anchor=W).pack(side=TOP, fill=X, expand=1)
        self.batt_label_var = StringVar()
        Label(self, textvariable=self.batt_label_var, font=self.VALUE_FONT, anchor=W).pack(side=TOP, fill=X, expand=1)
        self.ping_label_var = StringVar()
        Label(self, textvariable=self.ping_label_var, font=self.VALUE_FONT, anchor=W).pack(side=TOP, fill=X, expand=1)
        # Draw outline
        self.draw_outline()
        # Setup ROS
        if use_telemetry:
            rospy.Subscriber("/raposang/telemetry", RaposaTelemetry, self.handler_telemetry, queue_size=1)
        else:
            rospy.Subscriber("/imu/data", Imu, self.handler_imu, queue_size=1)
            rospy.Subscriber("/raposang/odometry", RaposaOdometry, self.handler_odometry, queue_size=1)
        rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)
        # Start ping
        self.launch_ping()
        # Main loop
        self.mainloop()

    def launch_ping(self):
        self.ping = subprocess.Popen(("ping", ROBOT_HOSTNAME), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        fcntl.fcntl(self.ping.stdout, fcntl.F_SETFL, os.O_NDELAY)
        self.pinger()

    PING_RE  = re.compile("bytes from.*time=([0-9.]+)")
    def pinger(self):
        while True:
            try:
                ln = self.ping.stdout.readline()
                re = self.PING_RE.search(ln)
                if re:
                    (cnt, rtt) = self.data["ping"] if self.data.has_key("ping") else (0, None)
                    self.data["ping"] = (cnt+1, float(re.group(1)))
                    self.update()
            except IOError:
                break
        self.after(100, self.pinger)

    def update(self):
        a = self.data["arm_angle"]
        q = self.data["orientation"]
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        phi = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
        the = math.asin(2*(q0*q2 - q3*q1))
        psi = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
        self.draw_single(0, -psi+math.pi/2, ARROW, value=psi+(2*math.pi if psi<0 else 0))
        self.draw_double(1, the-a, SIDE, the, ARM, value2=a)
        self.draw_single(2, phi, BACK)
        self.update_status()

    def handler_imu(self, msg):
        self.data["orientation"] = msg.orientation
        self.update()

    def handler_odometry(self, msg):
        self.data["arm_angle"] = math.radians(msg.arm_angle)
        self.update()

    def handler_telemetry(self, msg):
        self.data["orientation"] = msg.orientation
        self.data["arm_angle"] = math.radians(msg.arm_angle)
        self.data["wlan"] = (msg.wlan_link, msg.wlan_level, msg.wlan_noise)
        self.update()
        

    def draw_outline(self):
        for (i,l) in enumerate(LABELS):
            offset = i*(4*RADIUS+MARGIN)
            self.canvas.create_oval(offset+MARGIN, MARGIN, offset+MARGIN+2*RADIUS, MARGIN+2*RADIUS, dash=(1,2))
            if isinstance(l, str):
                self.canvas.create_text(offset+MARGIN+2*RADIUS+MARGIN, MARGIN,
                                        font=self.LABEL_FONT, text=l, anchor=NW)
            else:
                for (j,sl) in enumerate(l):
                    self.canvas.create_text(offset+MARGIN+2*RADIUS+MARGIN, MARGIN+j*3*MARGIN, text=sl, anchor=NW)
            self.canvas.create_line(offset+MARGIN+RADIUS, MARGIN, offset+MARGIN+RADIUS, MARGIN-6)
            self.canvas.create_line(offset+MARGIN+RADIUS, MARGIN+2*RADIUS, offset+MARGIN+RADIUS, MARGIN+2*RADIUS+6)
            self.canvas.create_line(offset+MARGIN, MARGIN+RADIUS, offset+MARGIN-6, MARGIN+RADIUS)
            self.canvas.create_line(offset+MARGIN+2*RADIUS, MARGIN+RADIUS, offset+MARGIN+2*RADIUS+6, MARGIN+RADIUS)


    def draw_single(self, index, angle, shape, subindex=0, value=None):
        if value is None:
            value = angle
        offset = index*(4*RADIUS+MARGIN)
        tag = "shape.%s.%s"%(index,subindex)
        s, c = math.sin(-angle), math.cos(-angle)
        R = np.array([[c, -s], [s, c]])
        points = np.array([MARGIN+offset+RADIUS, MARGIN+RADIUS]) + RADIUS*np.dot(shape, R.T)
        self.canvas.delete(tag)
        self.canvas.create_polygon(*points.flatten(), **dict(fill="#00A500", tags=tag))
        self.canvas.create_text(offset+MARGIN+2*RADIUS+MARGIN, 2*MARGIN+subindex*3*MARGIN,
                                font=self.VALUE_FONT, text=u"%+.1f\u00ba"%(math.degrees(value)), anchor=NW, tags=tag)

    def draw_double(self, index, angle1, shape1, angle2, shape2, value1=None, value2=None):
        self.draw_single(index, angle1, shape1, 0, value1)
        self.draw_single(index, angle2, shape2, 1, value2)

    def update_status(self):
        if self.data.has_key("wlan"):
            (link, level, noise) = self.data["wlan"]
            self.wlan_label_var.set("Wireless:  link=%d  level=%d dBm  noise=%d dBm"%(link, level, noise))
        if self.data.has_key("battery"):
            # TODO
            pass
        if self.data.has_key("ping"):
            (cnt, rtt) = self.data["ping"]
            sign = "-X" if cnt%2 else "X-"
            self.ping_label_var.set("Ping:  %s  %0.3f ms"%(sign, rtt))


if __name__=="__main__":
    Viewer().main()

# EOF
