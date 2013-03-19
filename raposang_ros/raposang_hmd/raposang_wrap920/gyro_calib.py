
import sys
import math
import pickle
import numpy as np
import numpy.linalg as la
from ransac import ransac

NODE_NAME = 'raposang_wrap920'
import roslib; roslib.load_manifest(NODE_NAME)
import rospy

from raposang_msgs.msg import *


def Q(l):
    (psi, theta, phi) = l
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sec_theta = 1/math.cos(theta)
    tan_theta = math.tan(theta)
    return np.array([[0, sin_phi*sec_theta, cos_phi*sec_theta],
                     [0, cos_phi,           -sin_phi],
                     [1, sin_phi*tan_theta, cos_phi*tan_theta]])



class LinearModel:
    
    def fit(self, data):
        return np.polyfit(data[:,0], data[:,1], 1)

    def get_error(self, data, model):
        #print "error:\n", model[1]*data[:,0] - model[0]
        y = model[0]*data[:,0] + model[1]
        return np.abs(y - data[:,1])


class Calib:
    counter = 0
    last_t = None
    Y = []
    X = []

    def calibrate_old(self):
        X = np.array(self.X)
        Y = np.array(self.Y)
        for i in xrange(3):
            self.dump_data("calib_%i.out"%(i), X[:,i], Y[:,i])
            print np.polyfit(X[:,i], Y[:,i], 1)

    def calibrate(self):
        with open("calib.gnuplot", "w") as fh:
            print >>fh, """
set xlabel "measured"
set ylabel "predicted"
"""
            X = np.array(self.X)
            Y = np.array(self.Y)
            model = LinearModel()
            for i in xrange(3):
                print "-- gyro", i, "--"
                data = np.vstack( (X[:,i], Y[:,i]) ).T
                self.dump_data("calib_%i.out"%(i), X[:,i], Y[:,i])
                try:
                    #          ransac(data, model,  n,    k,   t,   d, debug=False,return_all=False)
                    out, ret = ransac(data, model,  2, 1000,  50,   0.75*len(data), return_all=True)
                    #               n=min num data, k=max iter, t=error thresh, d=num close values to assert fit
                    inliers = ret["inliers"]
                    print len(inliers), "inliers in", len(data)
                    print "model:\n", out
                    print >>fh, """
set term x11 %d persist
set title "Axis %d"
plot [] [-250:250] "calib_%d.out" using 1:2 notitle,  (%f)*x+(%f) notitle
"""%(i, i, i, out[0], out[1])
                except ValueError:
                    print "Unable to perform regression"

    def dump_data(self, filename, x, y):
        with open(filename, "w") as fh:
            for (xi,yi) in zip(x,y):
                print >>fh, xi, yi

    def handler(self, msg):
        self.counter += 1
        tkk = msg.header.stamp.to_sec()
        lkk = np.array([msg.yaw, msg.pitch, msg.roll])
        wkk = np.array([msg.gyro_x, msg.gyro_y, msg.gyro_z])
        print self.counter, msg.header.stamp
        #
        if self.last_t is not None:
            Tk = tkk - self.last_t
            lk = self.last_l
            wk = self.last_w
            xk = wk
            yk = np.dot(la.inv(Q(lk)), lkk-lk) / Tk
            self.X.append(xk)
            self.Y.append(yk)
        #
        self.last_l = lkk
        self.last_w = wkk
        self.last_t = tkk
        if self.limit is not None and self.counter>=self.limit:
            self.calibrate()
            rospy.signal_shutdown("Done")

    def main(self, limit):
        self.limit = limit
        rospy.Subscriber("/monitor/rpy", ImuRPY, self.handler)
        rospy.loginfo("Node ready")
        rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)
        rospy.spin()






if __name__=="__main__":
    if len(sys.argv)>1:
        limit = int(sys.argv[1])
    else:
        limit = None
    Calib().main(limit)

    # EOF
