#! /usr/bin/env python

"""
usage: %(progname)s hostname
"""

PKG = 'raposang_panasonic_ip_camera' # this package name
NAME = 'camera'

import os, sys, string, time, getopt
import urllib

import roslib; roslib.load_manifest(PKG) 
import rospy 

from sensor_msgs.msg import CompressedImage, CameraInfo

import threading

class StreamThread(threading.Thread):
  def __init__(self, axis):
    threading.Thread.__init__(self)

    self.axis = axis
    self.setDaemon(True)

    self.notdone = True

    self.width = 640
    self.height = 480

  def stop(self):
    self.notdone = False

  def run(self):
    while self.notdone:
      try:
        self.stream()
      except:
        import traceback
        traceback.print_exc()
      time.sleep(1)

  def stream(self):
    url = 'http://%s/nphMotionJpeg' % self.axis.hostname
    url = url + "?resolultion=%dx%d" % (self.width, self.height)
    fp = urllib.urlopen(url)

    print 'Streaming \n'

    while self.notdone:
      boundary = fp.readline()

      header = {}
      while 1:
        line = fp.readline()
        if line == "\r\n": break
        line = line.strip()

        parts = line.split(": ", 1)
        header[parts[0]] = parts[1]

      content_length = int(header['Content-length'])

      img = fp.read(content_length)
      line = fp.readline()
      
      msg = CompressedImage()
      msg.header.stamp = rospy.Time.now()
      msg.format = "jpeg"
      msg.data = img

      self.axis.pub.publish(msg)

      cimsg = CameraInfo()
      cimsg.header.stamp = msg.header.stamp
      cimsg.width = self.width
      cimsg.height = self.height

      self.axis.caminfo_pub.publish(cimsg)

class Axis:
  def __init__(self, hostname, username, password):
    self.hostname = hostname
    self.username = username
    self.password = password

    self.st = None
    self.pub = rospy.Publisher("panasonic_ip_camera/compressed", CompressedImage, self)
    self.caminfo_pub = rospy.Publisher("panasonic_ip_camera/camera_info", CameraInfo, self)

  def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    if self.st is None:
      self.st = StreamThread(self)
      self.st.start()

  def peer_unsubscribe(self, topic_name, numPeers):
    if numPeers == 0 and self.st:
      self.st.stop()
      self.st = None


def server(hostname, username, password):
  rospy.init_node(NAME)

  axis = Axis(hostname, username, password)

  # spin() keeps Python from exiting until node is shutdown
  rospy.spin()


def test():
  pass

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0
  if len(args) == 0:
    usage(progname)
    return
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1

  if testflag:
    test()
    return

  if len(args) == 1:
    hostname = args[0]
    username = None
    password = None
  elif len(args) == 3:
    hostname = args[0]
    username = args[1]
    password = args[2]
  else:
    hostname = rospy.get_param('hostname', '192.168.0.253') 
    username = rospy.get_param('~username', 'root') 
    password = rospy.get_param('~password', '') 

  server(hostname, username, password)

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
