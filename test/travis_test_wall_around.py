#!/usr/bin/env python
import unittest, rostest
import rosnode, rospy
import time, sys
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger, TriggerResponse

class WallStopTest(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/wall_stop',nodes, "node does not exist")

    def set_sensor_values(self,lf,ls,rs,rf):
        with open("/dev/rtlightsensor0","w") as f:
            f.write("%d %d %d %d\n" % (rf,rs,ls,lf))

    def get_freqs(self):
        with open("/dev/rtmotor_raw_l0","r") as lf,\
             open("/dev/rtmotor_raw_r0","r") as rf:
            left = int(lf.readline().rstrip())
            right = int(rf.readline().rstrip())
        return left,right

    def test_io(self):
        left, right = self.set_and_get(400, 0, 0, 100)
        self.assertTrue(left == right == 0, "can't stop")

        left, right = self.set_and_get(0, 5, 1000, 0)
        self.assertTrue(left == right != 0, "stop wrongly by side sensors")

        left, right = self.set_and_get(0, 10, 0, 0) #curve to left
        self.assertTrue(left < right, "don't curve to left")

        left, right = self.set_and_get(0, 2000, 0, 0) #curve to right
        self.assertTrue(left > right, "don't curve to right")

        left, right = self.set_and_get(0, 5, 0, 0) # don't control when far from a wall
        self.assertTrue(0 < left == right, "curve wrongly")

if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('travis_test_wall_stop')
    rostest.rosrun('pimouse_run_corridor','travis_test_wall_stop',WallStopTest)
