#!/usr/bin/env python

""" read_terminal_output.py
Written by Lydia Zuehsow (Oktober13) c Fall 2017

When running hello_pixy.py from the libpixyusb library,
all outputted terminal output is piped to a constantly
updated text file (terminalDump.txt)

read_terminal_output reads from this text file and scrapes
relevant object and location data, publishing to a ros node.

Follow function (reading from a constantly updating file)
written by David M. Beazley.
His code is taken from http://www.dabeaz.com/generators/
"""

import time
import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from string import maketrans

class SeenObj(object):
    """Class template for any tracked object. Assumes each object has a unique singular "color signature" passed from PixyCam."""
    def __init__(self, passed_id = 0, passed_desc = 0, passed_x = 0, passed_y = 0, passed_width = 0, passed_height = 0):
        self.description = passed_desc
        self.id = passed_id
        self.x = passed_x
        self.y = passed_y
        self.width = passed_width
        self.height = passed_height
        self.lastseenframe = 0

class Tracker(object):
    """Initiates a tracker object, which handles all preprocessing of object coordinates before publishing to the appropriate ROS node."""
    def __init__(self):
        self.seenObjects = {}
        self.currframe = 0

    def interpretLine(self, passed_line):
        """Parses each line from the continually updating data text file, and extracts vital information."""
        dataDict = {"sig: " : "", "x: " : "", "y: " : "", "width: " : "", "height: " : ""}

        tempLine = passed_line
        if "frame" in tempLine:
            tempLine = tempLine.replace("frame ","")
            tempLine = tempLine.replace(":","")
            self.currframe = int(tempLine)
            
        elif "sig:" in tempLine:
            print tempLine
            for key in dataDict.keys():
                tempLine = tempLine.replace(key,dataDict[key])
            tempLine = tempLine.split(" ")
            print tempLine
            print ""
            obj = int(tempLine[2])
            if obj in self.seenObjects.keys():
                self.seenObjects[obj].x = int(float(tempLine[3]))
                self.seenObjects[obj].y = int(float(tempLine[4]))
                self.seenObjects[obj].width = int(float(tempLine[5]))
                self.seenObjects[obj].height = int(float(tempLine[6]))
                self.seenObjects[obj].lastseenframe = self.currframe
                self.publishInfo(obj)
            else:
                tempObject = SeenObj(obj,"Red object")
                self.seenObjects[obj] = tempObject
                print "Saw new object."
        return

    def publishInfo(self, obj):
        multilayout = MultiArrayLayout(
        dim = (MultiArrayDimension("XYWHF",5,1),MultiArrayDimension("XYWHF",5,1)),
        data_offset = 0
        )

        msg = Int16MultiArray(
            layout = multilayout,
            data = [self.seenObjects[obj].x, self.seenObjects[obj].y, self.seenObjects[obj].width, self.seenObjects[obj].height, self.seenObjects[obj].lastseenframe]
        )
        if (obj == 1) or (obj == 2):
            if obj == 1:
                quad_info.publish(msg)
                rostopic = "/redobject"
            elif obj == 2:
                obst_info.publish(msg)
                rostopic = "/blueobject"
            print "Published: ", (msg.data), " to ", rostopic
        else:
            print "Unidentified color signature."
        return

def follow(thefile):
    """Waits for updates to the continually updating text file of PixyCam data."""
    thefile.seek(0,2)
    while True:
        line = thefile.readline()
        if 0xFF == ord('q'):
            break
        if not line or ('\n' not in line):
            print "Slep"
            time.sleep(0.1)
            continue
        yield line
        # t.interpretLine(line)


if __name__ == '__main__':
    quad_info = rospy.Publisher('redobject', Int16MultiArray, queue_size=10)
    obst_info = rospy.Publisher('blueobject', Int16MultiArray, queue_size=10)
    rospy.init_node('pixynode')

    t = Tracker()

    logfile = open("terminalDump.txt","r")


    for line in follow(logfile):
        # print line
        t.interpretLine(line)