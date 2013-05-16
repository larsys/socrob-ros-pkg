#! /usr/bin/env python

"""
usage: %(progname)s hostname
"""

PKG = 'raposang_panasonic_ip_camera' # this package name
NAME = 'control'
HEADERS = {"Accept": "image/gif, image/x-xbitmap, image/jpeg, image/pjpeg, application/vnd.ms-powerpoint, application/vnd.ms-excel, application/msword, application/x-shockwave-flash, */*", "Accept-Language": "es", "Content-Type": "application/x-www-form-urlencoded", "User-Agent": "Mozilla/4.0 (compatible; MSIE 6.0; Windows NT 5.0; .NET CLR 1.1.4322; .NET CLR 2.0.50727)", "Connection": "Keep-Alive"}

hostname = []

import os, sys, string, time
import httplib, urllib

import roslib; roslib.load_manifest(PKG) 
import rospy 

from sensor_msgs.msg import CompressedImage, CameraInfo
from raposang_msgs.msg import PanasonicCameraControl

def control_camera(data):
  
  global hostname

  params = []
  check = True
  

  if data.turbo is True:
    pt = "2000"
    tt = "1875" 
    boost = 2
  else:
    pt = ""
    tt = ""
    boost = 1 
  
  if data.homePosition is True:
    params = urllib.urlencode ({"Direction":"HomePosition"})   
  elif data.panLeft is True:
    params = urllib.urlencode ({"Direction":"PanLeft%s" % (pt)})
  elif data.panRight is True:
    params = urllib.urlencode ({"Direction":"PanRight%s" % (pt)})
  elif data.tiltUp is True:
    params = urllib.urlencode ({"Direction":"TiltUp%s" % (tt)})
  elif data.tiltDown is True:
    params = urllib.urlencode ({"Direction":"TiltDown%s" % (tt)})
  elif data.zoomWide is True:
    params = urllib.urlencode ({"Direction":"ZoomWide", "Dist":1})   
  elif data.zoomTele is True:
    params = urllib.urlencode ({"Direction":"ZoomTele", "Dist":1})   
  elif data.mouse is True:
   x = data.x
   y = data.y
   params = urllib.urlencode ({"Direction":"Direct", "NewPosition.x":x, "NewPosition.y":y, "Width":"640", "Height":"480"})
  else:
    check = False;
    
  if check is True:
    conn = httplib.HTTPConnection ("192.168.0.253", 80)
    conn.request ("POST", "/nphControlCamera", params, HEADERS)

def main(argv, stdout, environ):
  
  global hostname
  global seconds
  
  hostname = rospy.get_param('hostname', '192.168.0.253')   
 
  rospy.init_node(NAME)
  rospy.Subscriber('IPCameraControl', PanasonicCameraControl, control_camera)

  rospy.spin()

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
