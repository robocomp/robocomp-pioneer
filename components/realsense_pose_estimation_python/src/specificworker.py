#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
import sys
from PySide2.QtCore import QTimer
from PySide2 import QtCore

from PySide2.QtWidgets import QApplication
from genericworker import *
import pyrealsense2 as rs
import numpy as np
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
import threading
import csv
import json
from datetime import datetime
from RoboCompGenericBase import *

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.Period = 50
        self.lock = threading.Lock()
        self.firsttime = False

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)


    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        self.num_cameras = params["num_cameras"]
        self.print = params["print"] == "false"
        self.cameras_dict = {}

        for i in range(int(self.num_cameras)) :
            device_serial = params["device_serial_"+str(i)]
            self.name = params["name_"+str(i)]
            rx = np.radians(float(params["rx_"+str(i)]))
            ry = np.radians(float(params["ry_"+str(i)]))
            rz = np.radians(float(params["rz_"+str(i)]))
            tx = float (params["tx_"+str(i)])
            ty = float(params["ty_"+str(i)])
            tz = float(params["tz_"+str(i)])
            
            # Transforms:   world -> slam_sensor -> robot -> origin 
            #                  (measure)     (sensor pose) (robot base wrt world origin)
            tm = TransformManager()
            tm.add_transform("robot", "origin",
                             pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0.0, 0.0, 0.0]),
                                                 [0.0, 0.0, 0.0]))

            tm.add_transform("slam_sensor", "robot",
                             pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([rx, ry, rz]),
                                                 [tx, ty, tz]))

            self.cameras_dict[self.name] = [device_serial, tm]

        # realsense configuration
        try:
            for key in self.cameras_dict:
                config = rs.config()
                config.enable_device(self.cameras_dict[key][0])
                config.enable_stream(rs.stream.pose)
                pipeline = rs.pipeline()
                
                
                #CARGA DEL JSON
                profile = config.resolve(pipeline)
                dev = profile.get_device()
                tm2 = dev.as_tm2()

                if(tm2):
                    print("*************************************")
                    pose_sensor = tm2.first_pose_sensor()
                    if key == "camera_back":
                        self.wheel_odometer_b = pose_sensor.as_wheel_odometer()
                    elif key == "camera_side":
                        self.wheel_odometer_s = pose_sensor.as_wheel_odometer()
                    try:
                        etc_path = key.replace("\"", "")+".json"
                        with open(etc_path, 'r') as the_file:
                            chars = []
                            for line in the_file: 
                                for c in line:
                                    chars.append(ord(c))  # char to uit8
                        #print("odo", chars) 
                        if key == "camera_back":
                            self.wheel_odometer_b.load_wheel_odometery_config(chars)
                        elif key == "camera_side":
                            self.wheel_odometer_s.load_wheel_odometery_config(chars)         
                    except OSError as err:
                        print("OS error: {0}".format(err))

                # load/configure wheel odometer
                #pipeline.start(config)   
                #pipeline.stop()
                pipeline.start(config)
                self.cameras_dict[key].append(pipeline)

        except Exception as e:
            print("Error initializing camera")
            print(e)
            sys.exit(-1)

        data_list=[]
        angle_list = []
        mapper_value = 0
        tracker_value = 0
        for key in self.cameras_dict:

            self.cameras_dict[key].append(data_list)
            self.cameras_dict[key].append(angle_list)
            self.cameras_dict[key].append(mapper_value)
            self.cameras_dict[key].append(tracker_value)

        return True


    @QtCore.Slot()
    def compute(self):

        Base = TBaseState()
        
        Base = self.differentialrobot_proxy.getBaseState()

        #self.genericbase_proxy.getBaseState(Base)
        

        for key in self.cameras_dict:

            v = rs.vector()
            v.z = -Base.advVz/1000
            v.x = 0
            v.y = 0
            if key == "camera_back":
                self.wheel_odometer_b.send_wheel_odometry(0, 0, v)
            elif key == "camera_side":
                self.wheel_odometer_s.send_wheel_odometry(0, 0, v)
            print("Velocidad", v.z, "\n")
            #self.wheel_odometer.send_wheel_odometry(0, 0, v)


            
            frames = self.cameras_dict[key][2].wait_for_frames()
            f = frames.first_or_default(rs.stream.pose)
            data = f.as_pose_frame().get_pose_data()
            

            #self.cameras_dict[key][3] = data
            print(key, data.translation.x * 1000.0, data.translation.z * 1000.0, data.translation.y * 1000.0,)

            self.cameras_dict[key][1].add_transform("world", "slam_sensor", pytr.transform_from_pq([data.translation.x * 1000.0,
                                                                                                    -data.translation.z * 1000.0,
                                                                                                    data.translation.y * 1000.0,
                                                                                                    data.rotation.w,
                                                                                                    data.rotation.x,
                                                                                                    data.rotation.y,
                                                                                                    data.rotation.z]))

    
            t = self.cameras_dict[key][1].get_transform("world", "origin")
            self.cameras_dict[key][3] = pytr.transform(t,[0,0,0,1])[0:3]
            self.cameras_dict[key][4] = self.quaternion_to_euler_angle(data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z)
            self.cameras_dict[key][5] = data.mapper_confidence
            self.cameras_dict[key][6] = data.tracker_confidence
            
            
            

        # print
        if self.print:
            ret = RoboCompFullPoseEstimation.FullPoseEuler()
            sigma = 0
            #CALCULATE ADDITION BOTH DATA'S CAMERA
            for key in self.cameras_dict:
                ret.x = ret.x + self.cameras_dict[key][3][0] * self.cameras_dict[key][6]
                ret.y = ret.y + self.cameras_dict[key][3][1] * self.cameras_dict[key][6]
                ret.z = ret.z + self.cameras_dict[key][3][2] * self.cameras_dict[key][6]
                ret.rx = ret.rx + self.cameras_dict[key][4][0] * self.cameras_dict[key][6]
                ret.ry = ret.ry + self.cameras_dict[key][4][1] * self.cameras_dict[key][6]
                ret.rz = ret.rz + self.cameras_dict[key][4][2] * self.cameras_dict[key][6]
                sigma = sigma + self.cameras_dict[key][6]
            #CALCULATE AVERAGE OF POSITION
            ret.x = ret.x / sigma
            ret.y = ret.y / sigma
            ret.z = ret.z / sigma
            
            #CALCULATE AVERAGE OF ANGLES
            ret.rx = ret.rx / sigma
            ret.ry = ret.ry / sigma
            ret.rz = ret.rz / sigma
            
            print("resultado", sigma, ret.x, ret.y, ret.z, ret.rx, ret.ry, ret.rz,"\n")


        

    def quaternion_to_euler_angle(self, w, x, y, z):

        #print(w,x,y,z)
        qx = x
        qy = y
        qz = z
        qw = w

        a1 = np.arctan2(2*qy*qw-2*qx*qz, 1- 2*qy*qy - 2*qz*qz)
        a2 = np.arcsin(2*qx*qy + 2*qz*qw)
        a0 = np.arctan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz)

        if np.isclose(qx*qy + qz*qw, 0.5):
            a1 = 2.0 * np.arctan2(qx,qw)
            a0 = 0.0
        if np.isclose(qx*qy + qz*qw, -0.5):
            a1 = -2.0 * np.arctan2(qx,qw)
            a0 = 0.0
        #return a0,a1,a2
        return a0,a2,a1
    
    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getFullPoseEuler method from FullPoseEstimation interface
    #
    def FullPoseEstimation_getFullPoseEuler(self):

        ret = RoboCompFullPoseEstimation.FullPoseEuler()
        sigma = 0
        #CALCULATE ADDITION BOTH DATA'S CAMERA
        for key in self.cameras_dict:
            ret.x = ret.x + self.cameras_dict[key][3][0] * self.cameras_dict[key][6]
            ret.y = ret.y + self.cameras_dict[key][3][1] * self.cameras_dict[key][6]
            ret.z = ret.z + self.cameras_dict[key][3][2] * self.cameras_dict[key][6]
            ret.rx = ret.rx + self.cameras_dict[key][4][0] * self.cameras_dict[key][6]
            ret.ry = ret.ry + self.cameras_dict[key][4][1] * self.cameras_dict[key][6]
            ret.rz = ret.rz + self.cameras_dict[key][4][2] * self.cameras_dict[key][6]
            sigma = sigma + self.cameras_dict[key][6]
        #CALCULATE AVERAGE OF POSITION
        ret.x = ret.x / sigma
        ret.y = ret.y / sigma
        ret.z = ret.z / sigma

        #CALCULATE AVERAGE OF ANGLES  (CHECK -PI to PI transition !!!!)
        ret.rx = ret.rx / sigma
        ret.ry = ret.ry / sigma
        ret.rz = ret.rz / sigma

        print("\r Device Position: ", ret.x, ret.y, ret.z, ret.rx, ret.ry,ret.rz, end="\r")
        return ret
    #
    # IMPLEMENTATION of getFullPoseMatrix method from FullPoseEstimation interface
    #
    def FullPoseEstimation_getFullPoseMatrix(self):
        ret = RoboCompFullPoseEstimation.FullPoseMatrix()
        t = self.tm.get_transform("origin", "slam_sensor")
        m = RoboCompFullPoseEstimation.FullPoseMatrix()
        m.m00 = t[0][0]
        m.m01 = t[0][1]
        m.m02 = t[0][2]
        m.m03 = t[0][3]
        m.m10 = t[1][0]
        m.m11 = t[1][1]
        m.m12 = t[1][2]
        m.m13 = t[1][3]
        m.m20 = t[2][0]
        m.m21 = t[2][1]
        m.m22 = t[2][2]
        m.m23 = t[2][3]
        m.m30 = t[3][0]
        m.m31 = t[3][1]
        m.m32 = t[3][2]
        m.m33 = t[3][3]
        return m
    #
    # IMPLEMENTATION of setInitialPose method from FullPoseEstimation interface
    #
    def FullPoseEstimation_setInitialPose(self, x, y, z, rx, ry, rz):

        for key in self.cameras_dict:
             self.cameras_dict[key][1].add_transform("robot", "origin",
                              pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([rx, ry, rz]),

                                                 [x, y, z]))

            # self.cameras_dict[key][3][0] = x
            # self.cameras_dict[key][3][1] = y
            # self.cameras_dict[key][3][2] = z
            # self.cameras_dict[key][4][0] = rx
            # self.cameras_dict[key][4][1] = ry
            # self.cameras_dict[key][4][2] = rz


    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompFullPoseEstimationPub you can publish calling this methods:
    # self.fullposeestimationpub_proxy.newFullPose(...)

    ######################
    # From the RoboCompFullPoseEstimation you can use this types:
    # RoboCompFullPoseEstimation.FullPoseMatrix
    # RoboCompFullPoseEstimation.FullPoseEuler

