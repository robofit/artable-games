#!/usr/bin/env python
# -*- coding: utf-8 -*-

##################################################################################
##
## @file       projector_calibrator.py
## @date       24. 03. 2018
## @brief      Projector calibration file inspired by art_projected_gui package
## @author     Juraj Bačovčin (xbacov04@stud.fit.vutbr.cz)
##
##################################################################################

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

## Class representing projector calibration
class ProjectorCalibrator():

    ## ProjectorCalibrator constructor
    def __init__(self, proj_id):

        self.proj_id = proj_id
        self.calibrated = None
        self.calibrated_cb = None
        self.calibrating = False

        self.proj_ns = "/art/" + proj_id + "/projector/"
        self.proj_calib_ns = "/art/interface/projected_gui/app/projectors_calibrated"

        self.calib_pub = rospy.Publisher(self.proj_calib_ns, Bool, queue_size=1, latch=True)
        self.calib_sub = rospy.Subscriber(self.proj_ns + "calibrated", Bool, self.calib_cb, queue_size=10)
        self.srv_calibrate = rospy.ServiceProxy(self.proj_ns + "calibrate", Trigger)

    ## Waiting until calibration service is available
    def wait_until_available(self):

        self.srv_calibrate.wait_for_service()

        while self.calibrated is None:
            rospy.sleep(0.1)

    ## Calibration definition
    def calibrate(self, calibrated_cb=None):

        if self.calibrating:
            return False

        if self.is_calibrated():
            if calibrated_cb is not None:
                calibrated_cb(self)
            return True

        self.calibrated_cb = calibrated_cb
        self.calibrating = True

        # Executing calibration service with ServiceException exception
        try:
            ret = self.srv_calibrate()
        except rospy.ServiceException:
            self.calibrating = False
            self.calibrated_cb = None
            return False

        if ret.success:
            rospy.loginfo("Starting projector calibration")
        else:
            rospy.logerr("Projector calibration failed")

        return ret.success

    ## Method returning value representing state of projector calibration (whether it is done or not)
    def is_calibrated(self):

        return self.calibrated

    ## Callback function invoked by subscriber calib_sub
    def calib_cb(self, msg):

        self.calibrated = msg.data
        self.calibrating = False

        if self.calibrated_cb is not None:
            self.calibrated_cb(self)
            self.calibrated_cb = None

##################################################################################
## End of file projector_calibrator.py
##################################################################################
