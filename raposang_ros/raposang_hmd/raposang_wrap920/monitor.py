
import math
import pickle
import numpy as np
import numpy.linalg as la
from Tkinter import *

NODE_NAME = 'raposang_wrap920'
import roslib; roslib.load_manifest(NODE_NAME)
import rospy

from std_msgs.msg import *
from geometry_msgs.msg import *
from raposang_msgs.msg import *
from visualization_msgs.msg import *

PARAM_FILENAME = "parameters.dat"

TOP_MARGIN = 20
LEFT_MARGIN = 100
HEIGHT = 20
WIDTH = 800
SPACING = 50

MIN_RANGE = 400


def versor(x):
    return x/la.norm(x)

def Rx(a):
    s, c = math.sin(a), math.cos(a)
    return np.array([[ 1,  0,  0],
                     [ 0,  c, -s],
                     [ 0,  s,  c]])

def Ry(a):
    s, c = math.sin(a), math.cos(a)
    return np.array([[ c,  0,  s],
                     [ 0,  1,  0],
                     [-s,  0,  c]])

def Rz(a):
    s, c = math.sin(a), math.cos(a)
    return np.array([[ c, -s,  0],
                     [ s,  c,  0],
                     [ 0,  0,  1]])

def R2q(R):
    T  = R.trace()
    ps = np.sqrt(np.array([1+2*R[0,0]-T, 1+2*R[1,1]-T, 1+2*R[2,2]-T, 1+T])) / 2
    #
    # print np.array([ps[0], (R[0,1] + R[1,0])/(4*ps[0]), (R[0,2] + R[2,0])/(4*ps[0]), (R[1,2] - R[2,1])/(4*ps[0])])
    # print np.array([(R[0,1] + R[1,0])/(4*ps[1]), ps[1], (R[1,2] + R[2,1])/(4*ps[1]), (R[2,0] - R[0,2])/(4*ps[1])])
    # print np.array([(R[0,2] + R[2,0])/(4*ps[2]), (R[1,2] + R[2,1])/(4*ps[2]), ps[2], (R[0,1] - R[1,0])/(4*ps[2])])
    # print np.array([(R[1,2] - R[2,1])/(4*ps[3]), (R[2,0] - R[0,2])/(4*ps[3]), (R[0,1] - R[1,0])/(4*ps[3]), ps[3]])
    #
    i = ps.argmax()
    p = ps[i]
    if i==0:
        return np.array([p, (R[0,1] + R[1,0])/(4*p), (R[0,2] + R[2,0])/(4*p), (R[1,2] - R[2,1])/(4*p)])
    elif i==1:
        return np.array([(R[0,1] + R[1,0])/(4*p), p, (R[1,2] + R[2,1])/(4*p), (R[2,0] - R[0,2])/(4*p)])
    elif i==2:
        return np.array([(R[0,2] + R[2,0])/(4*p), (R[1,2] + R[2,1])/(4*p), p, (R[0,1] - R[1,0])/(4*p)])
    elif i==3:
        return np.array([(R[1,2] - R[2,1])/(4*p), (R[2,0] - R[0,2])/(4*p), (R[0,1] - R[1,0])/(4*p), p])
    else:
        raise Exception, "Internal error"


def RollPitchYaw(R):
    beta = math.atan2(-R[2,0], math.sqrt(R[0,0]**2 + R[1,0]**2))
    cb = math.cos(beta)
    alpha = math.atan2(R[1,0]/cb, R[0,0]/cb)
    gamma = math.atan2(R[2,1]/cb, R[2,2]/cb)
    return np.array([gamma, beta, alpha])

class Viewer(Frame):
    LABELS = ["Mx", "My", "Mz",  "Ox", "Oy", "Oz",  "Ax", "Ay", "Az"]
    MODE   = [   0,    0,    0,     1,    1,    1,     0,    0,    0]
    BUF_SIZE = 1
    last_tag = "bar0"
    R0 = np.eye(3)

    def __init__(self):
        self.reset_calib()
        self.cf = ComplementaryFilter()
        self.master = Tk()
        Frame.__init__(self, self.master)

    def main(self):
        self.pack()
        # Configure master
        self.master.title("Wrap820 Monitor")
        self.master.protocol('WM_DELETE_WINDOW', self.quit)
        self.master.resizable(0, 0)
        # Create widgets
        self.status_var = StringVar()
        Label(self, textvariable=self.status_var, justify=LEFT).pack(side=BOTTOM, fill=X, expand=1, anchor=W)
        self.canvas = Canvas(self, width=1, height=1)
        #self.canvas = Canvas(self, width=LEFT_MARGIN+WIDTH+SPACING, height=TOP_MARGIN+SPACING*len(self.LABELS))
        self.canvas.pack(side=BOTTOM)
        Button(self, text="Quit", command=self.quit).pack(side=LEFT)
        Button(self, text="Reset calibration", command=self.reset_calib).pack(side=LEFT)
        Button(self, text="Reset attitude", command=self.reset_attitude).pack(side=LEFT)
        Button(self, text="(Re)Load parameters", command=self.try_load_params).pack(side=LEFT)
        Button(self, text="Save parameters", command=self.save_params).pack(side=LEFT)
        self.control_var = IntVar()
        Checkbutton(self, text="Control", variable=self.control_var).pack(side=RIGHT)
        self.debug_var = IntVar()
        Checkbutton(self, text="Debug", variable=self.debug_var, command=self.toggle_debug).pack(side=RIGHT)
        # Draw outline
        self.draw_outline()
        # Prepare values buffer
        self.buf = np.empty((self.BUF_SIZE, len(self.LABELS)))
        self.bix = 0
        # Try to load parameters
        self.try_load_params()
        # Setup ROS
        self.pub_pose  = rospy.Publisher("/monitor/pose",  PoseStamped)
        self.pub_cube  = rospy.Publisher("/monitor/cube",  Marker)
        self.pub_frame = rospy.Publisher("/monitor/frame", Marker)
        self.pub_rpy   = rospy.Publisher("/monitor/rpy", ImuRPY)
        self.pub_pan   = rospy.Publisher("/servo_pan", Float32)
        self.pub_tilt  = rospy.Publisher("/servo_tilt", Float32)
        rospy.Subscriber("/wrap920", ImuRaw, self.handler, queue_size=None)
        rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)
        # Main loop
        self.mainloop()

    def reset_calib(self):
        self.bounds = [None for l in self.LABELS]

    def reset_attitude(self):
        self.R0 = self.Rt

    def save_params(self):
        params = {}
        for k in ("R0", "bounds"):
            params[k] = getattr(self, k)
        with open(PARAM_FILENAME, "w") as fh:
            pickle.dump(params, fh)

    def try_load_params(self):
        try:
            with open(PARAM_FILENAME) as fh:
                params = pickle.load(fh)
                for (k,v) in params.items():
                    setattr(self, k, v)
                print "Loaded %d parameters from %s."%(len(params),PARAM_FILENAME)
        except IOError:
            print "WARNING: parameter file not found!"
            

    def handler(self, msg):
        self.buf[self.bix] = np.concatenate((msg.magnetometer, msg.gyroscope, msg.accelerometer))
        self.bix += 1
        if self.bix==self.BUF_SIZE:
            self.bix = 0
            self.update(msg.header.stamp, self.buf.mean(0))

    def update(self, stamp, values):
        for i in xrange(len(self.LABELS)):
            x = values[i]
            offset = SPACING*i
            if self.MODE[i]==0:
                if self.bounds[i] is None:
                    b = self.bounds[i] = [x, x]
                else:
                    b = self.bounds[i]
                    if x<b[0]: b[0]=x
                    if x>b[1]: b[1]=x

        self.bounds[6] = [-1050, 1050]
        self.bounds[7] = [-1050, 1050]
        self.bounds[8] = [-1050, 1050]

        if self.debug_var.get():
            self.draw_bars(stamp, values)
                
        bm = np.array(self.bounds[:3])
        ba = np.array(self.bounds[6:9])
        
        
        try:
            mag = 2.0*(values[:3]-bm[:,0])/(bm[:,1]-bm[:,0]) - 1
            acc = 2.0*(values[6:9]-ba[:,0])/(ba[:,1]-ba[:,0]) - 1

            acc *= (1,-1,1)
            mag *= (-1,1,1)

            R = np.empty((3,3))
            R[:,2] = e1 = versor(acc)
            R[:,0] = e2 = versor(mag - np.inner(mag,e1)*e1)
            R[:,1] = e3 = np.cross(e1,e2)
            R = R.T
            self.Rt = R

            #R = np.dot(R, self.R0.T)
            R = np.dot(self.R0.T, R)

            # for (i,e) in enumerate([e1,e2,e3]):
            for (i,e) in enumerate([R[:,0], R[:,1], R[:,2]]):
                # COLORS: Cyan, Magenta, Yellow
                msg = Marker(id=i, type=Marker.ARROW, action=Marker.ADD)
                msg.header.frame_id = "/world"
                msg.scale.x = 0.1
                msg.scale.y = 0.2
                color = [1, 1, 1, 1]
                color[i+1] = 0
                (msg.color.a, msg.color.r, msg.color.g, msg.color.b) = color
                v = (1 if (bm[i,1]-bm[i,0]>MIN_RANGE).all() else 0.1) * e
                msg.points = [Point(x=0, y=0, z=0), Point(x=v[0], y=v[1], z=v[2])]
                self.pub_frame.publish(msg)

            q = R2q(R.T)
            pose = PoseStamped()
            pose.header.frame_id = "/world"
            p, o = pose.pose.position, pose.pose.orientation
            (p.x, p.y, p.z) = (0, 0, 0)
            (o.x, o.y, o.z, o.w) = q
            self.pub_pose.publish(pose)
            cube = Marker(id=10, type=Marker.CUBE, action=Marker.ADD)
            cube.header.frame_id = "/world"
            p, o, s, c = cube.pose.position, cube.pose.orientation, cube.scale, cube.color
            (p.x, p.y, p.z) = (0, 0, 0)
            (s.x, s.y, s.z) = (1, 1, 1)
            (o.x, o.y, o.z, o.w) = q
            (c.a, c.r, c.g, c.b) = (0.9, 0.5, 0.5, 0.6)
            self.pub_cube.publish(cube)

            rpy = RollPitchYaw(R)
            self.cf.update(stamp.to_sec(), np.flipud(rpy), values[3:6])

            # Convert to degrees
            rpy = 180*rpy/np.pi
            msg = ImuRPY(roll=rpy[0], pitch=rpy[1], yaw=rpy[2],
                         gyro_x=values[3], gyro_y=values[4], gyro_z=values[5])
            msg.header.stamp = stamp
            self.pub_rpy.publish(msg)

            # Show in status bar
            self.status_var.set("roll=%3d  pitch=%3d  yaw=%3d"%(rpy[0], rpy[1], rpy[2]))

            # Control pan&tilt if requested
            if self.control_var.get():
                self.pub_pan.publish(90-rpy[2])
                self.pub_tilt.publish(90-rpy[1])

        except ZeroDivisionError:
            pass


    def toggle_debug(self):
        if self.debug_var.get():
            self.show_debug()
        else:
            self.hide_debug()

    def show_debug(self):
        self.canvas.configure(width=LEFT_MARGIN+WIDTH+SPACING, height=TOP_MARGIN+SPACING*len(self.LABELS))

    def hide_debug(self):
        self.canvas.configure(width=1, height=1)

    def draw_outline(self):
        for (i,l) in enumerate(self.LABELS):
            offset = SPACING*i
            self.canvas.create_text(LEFT_MARGIN-20, TOP_MARGIN+(HEIGHT/2)+offset, text=l, anchor=E)
            self.canvas.create_rectangle(LEFT_MARGIN-1, TOP_MARGIN+offset-1,
                                         LEFT_MARGIN+WIDTH, TOP_MARGIN+HEIGHT+offset)
            self.canvas.create_line(LEFT_MARGIN+WIDTH/2, TOP_MARGIN+offset-10,
                                    LEFT_MARGIN+WIDTH/2, TOP_MARGIN+offset+HEIGHT)

    def draw_bars(self, stamp, values):
        tag = "bar1" if self.last_tag=="bar0" else "bar0"
        for i in xrange(len(self.LABELS)):
            x = values[i]
            b = self.bounds[i]
            offset = SPACING*i
            if self.MODE[i]==0:
                width = x/2 + WIDTH/2
                self.canvas.create_rectangle(LEFT_MARGIN, TOP_MARGIN+offset,
                                             LEFT_MARGIN+width, TOP_MARGIN+offset+HEIGHT/2,
                                             width=0, fill="green", tags=tag)
                self.canvas.create_text(LEFT_MARGIN, TOP_MARGIN+offset+HEIGHT+5,
                                    text="min: %s"%(b[0]), tags=tag, anchor=NW)
                self.canvas.create_text(LEFT_MARGIN+WIDTH, TOP_MARGIN+offset+HEIGHT+5,
                                        text="max: %s"%(b[1]), tags=tag, anchor=NE)
                self.canvas.create_text(LEFT_MARGIN+WIDTH/2, TOP_MARGIN+offset+HEIGHT+5,
                                        text="range: %s"%(b[1]-b[0]),
                                        fill="DarkGreen" if b[1]-b[0]>MIN_RANGE else "DarkRed",
                                        tags=tag, anchor=N)
                if b[0]!=b[1]:
                    u = 1.0*(x-b[0])/(b[1]-b[0])
                    self.canvas.create_rectangle(LEFT_MARGIN, TOP_MARGIN+offset+HEIGHT/2,
                                                 LEFT_MARGIN+round(WIDTH*u), TOP_MARGIN+offset+HEIGHT,
                                                 width=0, fill="red", tags=tag)
            elif self.MODE[i]==1:
                width = x/2 + WIDTH/2
                self.canvas.create_rectangle(LEFT_MARGIN, TOP_MARGIN+offset,
                                             LEFT_MARGIN+width, TOP_MARGIN+offset+HEIGHT,
                                             width=0, fill="green", tags=tag)
            else:
                raise Exception, "Internal error"
        self.canvas.delete(self.last_tag)
        self.last_tag = tag




def Q(l):
    (psi, theta, phi) = l
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sec_theta = 1/math.cos(theta)
    tan_theta = math.tan(theta)
    return np.array([[0, sin_phi*sec_theta, cos_phi*sec_theta],
                     [0, cos_phi,           -sin_phi],
                     [1, sin_phi*tan_theta, cos_phi*tan_theta]])

class ComplementaryFilter:
    last_y = None

    def __init__(self):
        self.l = np.zeros(3)
        self.b = np.zeros(3)

    def update(self, t, y, w):
        """y is yaw-pitch-roll, w is x-y-z"""
        if self.last_y is not None:
            T = t - self.last_t
            print "-- real delta y:", y-self.last_y
            print "   pred delta y:", T*np.dot(Q(y), w)

        self.last_y = y
        self.last_t = t



if __name__=="__main__":
    Viewer().main()

# EOF
