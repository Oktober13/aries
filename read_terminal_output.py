#!/usr/bin/env python

""" read_terminal_output.py
Written by Lydia Zuehsow (Oktober13) c Fall 2017 - Spring 2018
lydiazuehsow.weebly.com

Runs hello_pixy.py (libpixyusb library) and pipes all outputted
data to a constantly updated text file. (terminalDump.txt)

read_terminal_output then reads from this text file and scrapes
relevant object and location data, publishing to a ros node.

Follow function (reading from a constantly updating file)
written by David M. Beazley.
His code is taken from http://www.dabeaz.com/generators/
"""

import argparse
import atexit #Used for cleanup at shutdown
import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from string import maketrans
import subprocess #Used for subprocesses running in background
import time #used to sync transmissions

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
    def __init__(self, verbosity,rospub1,rospub2):
        self.seenObjects = {}
        self.currframe = 0
        self.verbose = verbosity
        self.quad_info = rospub1
        self.obst_info = rospub2
        self.newdata = False

    def readfile(self, filename, process, verbosity):
        """Wrapper function that repeatedly calls follow() to get updates from the piped text dump .txt file."""
        logfile = open(filename,"r")

        for line in self.follow(logfile, process, verbosity):
            self.interpretLine(line)
        return

    def follow(self, thefile, process, verbose):
        """Waits for updates to the continually updating text file of PixyCam data."""
        thefile.seek(0,2)
        try:
            while True:
                line = thefile.readline()
                if not line or ('\n' not in line):
                    if verbose and self.newdata is True: 
                        print "No new data."
                        self.newdata = False
                    continue
                self.newdata = True
                yield line
                # t.interpretLine(line)
            return
        except KeyboardInterrupt: 
            atexit.register(cleanup,process)
            return

    def interpretLine(self, passed_line):
        """Parses each line from the continually updating data text file, and extracts vital information."""
        dataDict = {"sig: " : "", "x: " : "", "y: " : "", "width: " : "", "height: " : ""}

        tempLine = passed_line
        if "frame" in tempLine:
            tempLine = tempLine.replace("frame ","")
            tempLine = tempLine.replace(":","")
            self.currframe = int(tempLine)
            
        elif "sig:" in tempLine:
            if self.verbose:
                print tempLine

            for key in dataDict.keys():
                tempLine = tempLine.replace(key,dataDict[key])

            tempLine = tempLine.split(" ")
            if self.verbose:
                print tempLine
                print ""
            objnum = int(tempLine[2])
            if objnum in self.seenObjects.keys():
                self.seenObjects[objnum].x = int(float(tempLine[3]))
                self.seenObjects[objnum].y = int(float(tempLine[4]))
                self.seenObjects[objnum].width = int(float(tempLine[5]))
                self.seenObjects[objnum].height = int(float(tempLine[6]))
                self.seenObjects[objnum].lastseenframe = self.currframe

                self.publishInfo(objnum)
            else:
                tempObject = SeenObj(objnum,"Red object")
                self.seenObjects[objnum] = tempObject
                print "Saw new object."
        return

    def publishInfo(self, objnum):
        """Publishes scraped .txt data to appropriate rosnode for use elsewhere."""

        msg = Int16MultiArray(
            data = [self.seenObjects[objnum].x, self.seenObjects[objnum].y, self.seenObjects[objnum].width, self.seenObjects[objnum].height, self.seenObjects[objnum].lastseenframe]
        )

        if (objnum == 1) or (objnum == 2):
            if objnum == 1:
                self.quad_info.publish(msg)
                rostopic = "/redobject"
            if objnum == 2:
                self.obst_info.publish(msg)
                rostopic = "/blueobject"
            if self.verbose:
                print "Published: ", (msg.data), " to ", rostopic
        else:
            if self.verbose:
                print "Unidentified color signature."
        return

def main():
    parser = argparse.ArgumentParser(description='Output processed data read from a PixyCam.')
    parser.add_argument("-v", "--verbose", help="Print all received data to terminal.", action = "store_true")
    args = parser.parse_args()

    print args.verbose

    if args.verbose:
        print "Verbose text output has been selected."
    else:
        print "Text output has been silenced."

     # Rospy was intercepting users' KeyboardInterrupt, so I'm handling rosnode shutdown manually in the cleanup function instead.
    rospy.init_node('pixynode', disable_signals=True)
    quad_info = rospy.Publisher('redobject', Int16MultiArray, queue_size=10)
    obst_info = rospy.Publisher('blueobject', Int16MultiArray, queue_size=10)

    file = "terminalDump.txt"
    f = open(file, "w")
    t = Tracker(args.verbose,quad_info,obst_info)

    # tempstring = './hello_pixy >> ' + file
    proc = subprocess.Popen(['./hello_pixy'], stdin=None, stdout=f, stderr=None, close_fds=True)
    t.readfile(file, proc, args.verbose)

def cleanup(process):
    rospy.signal_shutdown("User triggered KeyboardInterrupt.")
    process.kill()
    print "Process finished cleanly."
    return

if __name__ == '__main__':
    main()