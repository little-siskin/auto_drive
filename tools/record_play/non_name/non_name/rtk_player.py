#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Generate Planning Path
"""

import argparse
import os
import rospy
import sys
from numpy import genfromtxt
import scipy.signal as signal
import atexit
import logging
from logger import Logger

from modules.canbus.proto import chassis_pb2
from modules.control.proto import pad_msg_pb2
from modules.hmi.proto import runtime_status_pb2
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_pb2

# Import hmi_status_helper
APOLLO_ROOT = os.path.join(os.path.dirname(__file__), '../../../')
hmi_utils_path = os.path.join(APOLLO_ROOT, 'modules/hmi/utils')
log_dir = os.path.dirname(
        os.path.abspath(__file__)) + "/../../../data/log/"
if hmi_utils_path not in sys.path:
    sys.path.append(hmi_utils_path)
import hmi_status_helper

import os.path as op
from path_manager import PathManager
SEARCH_INTERVAL = 1000


class RtkPlayer(object):
    """
    rtk player class
    """


    def __init__(self, record_file, speedmultiplier, completepath, replan):
        """Init player."""
        self.firstvalid = False
        self.logger = Logger.get_logger(tag="RtkPlayer")
        self.logger.info("Load record file from: %s" % record_file)
        try:
            file_handler = open(record_file, 'r')
        except:
            self.logger.error("Cannot find file: " + record_file)
            file_handler.close()
            sys.exit(0)

        self.data = genfromtxt(file_handler, delimiter=',', names=True)
        file_handler.close()

        self.localization = localization_pb2.LocalizationEstimate()
        self.chassis = chassis_pb2.Chassis()
        self.padmsg = pad_msg_pb2.PadMessage()
        self.localization_received = False
        self.chassis_received = False

        self.planning_pub = rospy.Publisher(
            '/apollo/planning', planning_pb2.ADCTrajectory, queue_size=1)

        self.speedmultiplier = speedmultiplier / 100
        self.terminating = False
        self.sequence_num = 0

        b, a = signal.butter(6, 0.05, 'low')
        self.data['acceleration'] = signal.filtfilt(b, a,
            self.data['acceleration'])

        self.start = 0
        self.end = 0
        self.closestpoint = 0
        self.automode = False

        self.replan = (replan == 't')
        self.completepath = (completepath == 't')

        self.estop = False

        # Report status to HMI.
        status_pb = runtime_status_pb2.RuntimeStatus()
        status_pb.tools.planning_ready = True
        hmi_status_helper.HMIStatusHelper.report_status(status_pb)

        self.logger.info("Planning Ready")

    def localization_callback(self, data):
        """
        New localization Received
        """
        self.localization.CopyFrom(data)
        self.carx = self.localization.pose.position.x
        self.cary = self.localization.pose.position.y
        self.carz = self.localization.pose.position.z
        self.cartheta = self.localization.pose.heading
        self.localization_received = True

    def chassis_callback(self, data):
        """
        New chassis Received
        """
        self.chassis.CopyFrom(data)
        self.automode = (self.chassis.driving_mode ==
                         chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE)
        self.chassis_received = True

    def padmsg_callback(self, data):
        """
        New message received
        """
        if self.terminating == True:
            self.logger.info("terminating when receive padmsg")
            return

        self.padmsg.CopyFrom(data)

    def restart(self):
        shortest_dist_sqr = float('inf')
        self.logger.info("before replan self.start=%s, self.closestpoint=%s" %
                         (self.start, self.closestpoint))
        search_start = max(self.start - SEARCH_INTERVAL / 2, 0)
        search_end = min(self.start + SEARCH_INTERVAL / 2, len(self.data))
        for i in range(search_start, search_end):
            dist_sqr = (self.carx - self.data['x'][i]) ** 2 + \
                   (self.cary - self.data['y'][i]) ** 2
            if dist_sqr <= shortest_dist_sqr:
                self.start = i
                shortest_dist_sqr = dist_sqr

        self.closestpoint = self.start
        self.starttime = rospy.get_time()
        self.logger.info("finish replan at time %s, self.closestpoint=%s" %
                         (self.starttime, self.closestpoint))

    def publish_planningmsg(self):
        """
        Generate New Path
        """
        if not self.localization_received:
            self.logger.warning(
                "locaization not received yet when publish_planningmsg")
            return

        planningdata = planning_pb2.ADCTrajectory()
        now = rospy.get_time()
        planningdata.header.timestamp_sec = now
        planningdata.header.module_name = "planning"
        planningdata.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1

        self.logger.debug(
            "publish_planningmsg: before adjust start: self.start = %s, self.end=%s"
            % (self.start, self.end))
        if self.replan or self.sequence_num <= 1 or not self.automode:
            self.logger.info(
                "trigger replan: self.replan=%s, self.sequence_num=%s, self.automode=%s"
                % (self.replan, self.sequence_num, self.automode))
            self.restart()
        else:
            time_elapsed = now - self.starttime
            time_diff = self.data['time'][self.start] - \
                self.data['time'][self.closestpoint]
            while time_diff < time_elapsed and self.start < (
                    len(self.data) - 1):
                self.start = self.start + 1
                time_diff = self.data['time'][self.start] - \
                    self.data['time'][self.closestpoint]

        xdiff_sqr = (self.data['x'][self.start] - self.carx)**2
        ydiff_sqr = (self.data['y'][self.start] - self.cary)**2
        if xdiff_sqr + ydiff_sqr > 4.0:
            self.logger.info("trigger replan: distance larger than 2.0")
            self.restart()

        if self.completepath:
            self.start = 0
            self.end = len(self.data) - 1
        else:
            self.start = max(self.start - 100, 0)
            self.end = min(self.start + 1000, len(self.data) - 1)

        self.logger.debug(
            "publish_planningmsg: after adjust start: self.start = %s, self.end=%s"
            % (self.start, self.end))
        for i in range(self.start, self.end):
            adc_point = planning_pb2.ADCTrajectoryPoint()
            adc_point.x = self.data['x'][i]
            adc_point.y = self.data['y'][i]
            adc_point.z = self.data['z'][i]
            adc_point.speed = self.data['speed'][i] * self.speedmultiplier
            adc_point.acceleration_s = self.data['acceleration'][
                i] * self.speedmultiplier
            adc_point.curvature = self.data['curvature'][i]
            adc_point.curvature_change_rate = self.data[
                'curvature_change_rate'][i]

            time_diff = self.data['time'][i] - \
                self.data['time'][self.closestpoint]

            adc_point.relative_time = time_diff / self.speedmultiplier - (
                now - self.starttime)

            adc_point.theta = self.data['theta'][i]
            adc_point.accumulated_s = self.data['s'][i]

            planningdata.adc_trajectory_point.extend([adc_point])

        planningdata.estop.is_estop = self.estop

        planningdata.total_path_length = self.data['s'][self.end] - \
            self.data['s'][self.start]
        planningdata.total_path_time = self.data['time'][self.end] - \
            self.data['time'][self.start]
        planningdata.gear = int(self.data['gear'][self.start])

        self.planning_pub.publish(planningdata)
        self.logger.debug("Generated Planning Sequence: " +
                          str(self.sequence_num - 1))

    def shutdown(self):
        """
        shutdown rosnode
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        rospy.sleep(0.2)

    def quit(self, signum, frame):
        """
        shutdown the keypress thread
        """
        sys.exit(0)


def main():
    """
    Main rosnode
    """

    """
    garage_path = log_dir + 'garage.csv'
    dst_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/conf.yaml'
    AB_Gener = PathGener(garage_path,conf_path)
    AB_Gener.generate_path_file(dst_path)
    """
    garage_path = log_dir + 'garage.csv'
    #garage_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    dst_muti_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/path_conf.yaml'
    manager_conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/manager_conf.yaml'
    MutiAB_Manager = PathManager(garage_path,conf_path,manager_conf_path)
    MutiAB_Manager.generate_muti_path_file(dst_muti_path,MutiAB_Manager.get_lines_num())


    parser = argparse.ArgumentParser(
        description='Generate Planning Trajectory from Data File')
    parser.add_argument(
        '-s',
        '--speedmulti',
        help='Speed multiplier in percentage (Default is 100) ',
        type=float,
        default='100')
    parser.add_argument(
        '-c', '--complete', help='Generate complete path (t/F)', default='F')
    parser.add_argument(
        '-r',
        '--replan',
        help='Always replan based on current position(t/F)',
        default='F')
    args = vars(parser.parse_args())

    rospy.init_node('rtk_player', anonymous=True)

    Logger.config(
        log_file=os.path.join(APOLLO_ROOT, 'data/log/rtk_player.log'),
        use_stdout=True,
        log_level=logging.DEBUG)

    #record_file = os.path.join(APOLLO_ROOT, 'data/log/garage.csv')
    record_file = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    
    player = RtkPlayer(record_file, args['speedmulti'],
                       args['complete'].lower(), args['replan'].lower())
    atexit.register(player.shutdown)

    rospy.Subscriber('/apollo/canbus/chassis', chassis_pb2.Chassis,
                     player.chassis_callback)

    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     player.localization_callback)

    rospy.Subscriber('/apollo/control/pad', pad_msg_pb2.PadMessage,
                     player.padmsg_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        player.publish_planningmsg()
        rate.sleep()


if __name__ == '__main__':
    main()
