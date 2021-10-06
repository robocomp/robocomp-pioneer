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
from PySide2.QtCore import QMutex, QMutexLocker
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import time
import serial
from RoboCompRadar import *
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
        self.Period = 50
        self.distance=0.0
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        try:
            uart_port = params["uart_port"]
            baudrate = int(params["baudrate"])
            timeout= int(params["timeout"])
        except:
            traceback.print_exc()
            print("Error reading config params")
        self.ser = serial.Serial(
            port=uart_port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )
        print(self.ser)
        return True


    @QtCore.Slot()
    def compute(self):
        stringDistance = self.ser.readline().decode(encoding="utf-8")
        if len(stringDistance) > 0:
            self.distance = float(stringDistance)
            print("distancia", self.distance)

        return True

    def startup_check(self):
        print(f"Testing RoboCompRadar.RadarData from ifaces.RoboCompRadar")
        test = ifaces.RoboCompRadar.RadarData()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getMonitorState method from MonitorBase interface
    #
    def MonitorBase_getMonitorState(self):
        ret = RoboCompMonitorBase.MonitorStates()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getData method from Radar interface
    #
    def Radar_getData(self):
        ret = ifaces.RoboCompRadar.RadarData()
        ret.distance=self.distance
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompRadar you can use this types:
    # RoboCompRadar.RadarData


