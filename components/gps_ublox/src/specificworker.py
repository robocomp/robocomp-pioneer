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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import numpy as np
import timeit
from datetime import datetime
import utm
from gps3 import gps3
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
from pyproj import Proj, transform, Transformer
import interfaces as ifaces

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        self.theta = 0.27203221648765 + 3.1416  # Ajuste de orientación GPS a mapa DSR
        self.phi = 0
        # Build rotation matrix
        rot = np.array([
            [np.cos(self.theta), -np.sin(self.theta), 0.0, 0.0],
            [np.sin(self.theta), np.cos(self.theta), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        )
        # Build shear/skew matrix
        m = np.tan(self.phi)
        skew = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [m, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        )
        # get affine transform
        self.a = rot @ skew
        # Build pipeline
        self.pt_transform = Transformer.from_pipeline(
            f"+proj=pipeline "
            # f"+step +proj=affine +xoff={origin_x} +yoff={origin_y} "
            f"+step +proj=affine +xoff={0} +yoff={0} "
            f"+s11={self.a[0, 0]} +s12={self.a[0, 1]} +s13={self.a[0, 2]} "
            f"+s21={self.a[1, 0]} +s22={self.a[1, 1]} +s23={self.a[1, 2]} "
            f"+s31={self.a[2, 0]} +s32={self.a[2, 1]} +s33={self.a[2, 2]} "
        )
        self.gps_socket = gps3.GPSDSocket()
        self.data_stream = gps3.DataStream()
        self.gps_socket.connect()
        self.gps_socket.watch()
        self.p1 = Proj("epsg:4326")
        self.p2 = Proj("epsg:23030")
        self.fixedX =473301.52163559836#473300.2903119924 #473301.52163559836  473297.9574126723 +3.543089878678322
        self.fixedY =-4408256.16668115#-4408257.945248222 #-4408256.16668115  -4408257.768826621 -1.351344217300415
        self.originX = 0
        self.originY = 0
        return True


    @QtCore.Slot()
    def compute(self):
        self.gps_socket.next()
        if (self.gps_socket.response):
            #print("aqui", self.gps_socket.response)
            self.data_stream.unpack(self.gps_socket.response)
            self.altitude = self.data_stream.TPV['alt']
            self.latitude = self.data_stream.TPV['lat']
            self.longitude = self.data_stream.TPV['lon']

            if self.data_stream.TPV['lat'] != 'n/a':

                u = utm.from_latlon(self.latitude, self.longitude)
                self.xgps2 = u[0]
                self.ygps2 = u[1]


                #transform points into UTM
                self.UTMx = self.xgps2
                self.UTMy = self.ygps2
                self.xgps2, self.ygps2 = self.pt_transform.transform(
                    self.xgps2, self.ygps2)
                

                self.xgps2=(self.xgps2*1000 - (self.fixedX*1000) + (self.originX)) #- 3543.089878678322)
                self.ygps2=(self.ygps2*1000 - (self.fixedY*1000) + (self.originY)) #- 1351.344217300415)
                print("X:",self.xgps2)
                print("Y:", self.ygps2)

        
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)



    def GpsUblox_getData(self):
        ret = ifaces.RoboCompGpsUblox.DatosGPS()
        ret.altitude = self.altitude
        ret.latitude = self.latitude
        ret.longitude = self.longitude
        ret.UTMx = self.UTMx
        ret.UTMy = self.UTMy
        ret.mapx = self.xgps2
        ret.mapy = self.ygps2

        return ret
        
    def GpsUblox_setInitialPose(self, x, y):
    
        self.originX=x
        self.originY=y
        
        pass

 

    #
    # IMPLEMENTATION of getMonitorState method from MonitorBase interface
    #
    def MonitorBase_getMonitorState(self):
        ret = ifaces.RoboCompMonitorBase.MonitorStates()
        #
        # write your CODE here
        #
        return ret
    # ===================================================================
    # ===================================================================

 
    ######################
    # From the RoboCompGpsUblox you can publish calling this methods:
    # self.gpsublox_proxy.getData(...)

    ######################
    # From the RoboCompGpsUblox you can use this types:
    # RoboCompGpsUblox.DatosGPS

    ######################
    # From the RoboCompFullPoseEstimation you can call this methods:
    # self.fullposeestimation_proxy.getFullPoseEuler(...)
    # self.fullposeestimation_proxy.getFullPoseMatrix(...)
    # self.fullposeestimation_proxy.setInitialPose(...)

    ######################
    # From the RoboCompFullPoseEstimation you can use this types:
    # RoboCompFullPoseEstimation.FullPoseMatrix
    # RoboCompFullPoseEstimation.FullPoseEuler

    ######################
    # From the RoboCompGpsUblox you can publish calling this methods:
    # self.gpsublox_proxy.getData(...)
    # self.gpsublox_proxy.setInitialPose(...)

    ######################
    # From the RoboCompGpsUblox you can use this types:
    # RoboCompGpsUblox.DatosGPS

    ######################
    # From the RoboCompGpsUblox you can use this types:
    # RoboCompGpsUblox.DatosGPS


