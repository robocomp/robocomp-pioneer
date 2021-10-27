#!/bin/bash

sleep 2      

TERMINAL_ID_0=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 0)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 0 "DSR"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_0"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_0"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_0"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_0"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_0"

SESSION_ID_1=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_1=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 1)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 1 "Pioneer"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_1"


###### TAB LASER
SESSION_ID_2=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_2=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 2)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 2 "Pioneer-Laser"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_2"

###### TAB GPS
SESSION_ID_3=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_3=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 3)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 3 "Pioneer-Localizacion"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_3"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_3"


###### TAB CAMARAS

SESSION_ID_4=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_4=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 4)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 4 "Pioneer-Cameras"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_4"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_4"




qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 "ssh -X pioneernuc@pioneernuc.local"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 8 "ssh -X pioneernuc@pioneernuc.local"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 9 "ssh -X pioneernuc@pioneernuc.local"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 11 "ssh -X pioneernuc@pioneernuc.local"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 13 "ssh -X pioneernuc@pioneernuc.local"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 14 "ssh -X pioneernuc@pioneernuc.local"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 15 "ssh -X pioneernuc@pioneernuc.local"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 17 "ssh -X pioneernuc@pioneernuc.local"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 18 "ssh -X pioneernuc@pioneernuc.local"




##GPS
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 15 "killall -9 python3"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 15 "cd robocomp/components/robocomp-pioneer/components/gps_ublox/"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 15 "src/gps_ublox.py etc/config"

##PLOT GPS
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 17 "cd robocomp/components/robocomp-pioneer/components/realsensePoseEstimation/"

##Realsense SLAM
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 17 "cd robocomp/components/robocomp-pioneer/components/realsensePoseEstimation/"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 17 "bin/realSensePoseEstimation etc/config_side"

##Pioneer-Viewer
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 16 "cd robocomp/components/robocomp-robolab/components/viewers/pioneer_viewer/"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 16 "bin/giraff_viewer etc/config"

#BASE PIONEER
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 "killall -9 pioneer"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 "cd ~/robocomp/components/robocomp-pioneer/components/pioneer_robot"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 "bin/pioneer etc/config"

#CAMERA VIEWER
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 10 "cd robocomp/components/robocomp-robolab/components/viewers/camera_simple_viewer/"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 10 "bin/camera_simple_viewer etc/config"

#LASER I
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 12 "cd ~/robocomp/components/robocomp-pioneer/Agents/mission_controller_pioneer"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 12 "bin/mission_controller_pioneer etc/config"

#LASER 2
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 14 "cd ~/robocomp/components/robocomp-pioneer/components/hokuyo_python"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 14 "src/hokuyo_python.py etc/config_front"

#LASER INTEGRATOR
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 13 "cd ~/robocomp/components/robocomp-pioneer/components/laser-integrator"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 13 "bin/laser_integrator etc/config"

#GPS

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 9 "cd ~/robocomp/components/robocomp-pioneer/components/radar"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 9 "src/radar.py etc/config"

#USB CAMERA

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 11 "cd ~/robocomp/components/robocomp-pioneer/components/usb_camera"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 11 "bin/usb_camera etc/config"

###MONITOR
 
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 8 "rcnode &"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 8 "cd ~/robocomp/components/robocomp-pioneer/components/pioneer_monitor"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 8 "bin/pioneer_monitor etc/config"


#DSR

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 1 "cd ~/robocomp/components/dsr-graph/components/idserver"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 1 "bin/idserver etc/config_pioneer"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "cd ~/robocomp/components/robocomp-pioneer/Agents/pioneer_dsr"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "bin/pioneer_dsr etc/config_robot"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 5 "cd ~/robocomp/components/dsr-graph/components/path_follower"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 5 "bin/path_follower etc/config_pioneer_real"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 4 "cd ~/robocomp/components/dsr-graph/components/webengine-ui"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 4 "bin/webengine_ui etc/config"


qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 3 "cd ~/robocomp/components/dsr-graph/components/elastic_band"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 3 "bin/elastic_band etc/config_pioneer"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 2 "cd ~/robocomp/components/dsr-graph/components/path_planner_astar"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 2 "bin/path_planner_astar etc/config_exterior-robolab"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 6 "cd robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 6 "bin/JoystickPublish etc/config_pioneer"


#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 6 "echo 6"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 "echo 7"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 8 "echo 8"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 9 "echo 9"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 10 "echo 10"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 11 "echo 11"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 12 "echo 12"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 13 "echo 13"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 14 "echo 14"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 15 "echo 15"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 16 "echo 16"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 17 "echo 17"


###Gran angular
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 18 "cd ~/robocomp/components/robocomp-robolab/components/hardware/camera/three_vertical_realsense_camera"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 18 "bin/three_vertical_realsense_camera etc/config"

sleep 7
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 19 "cd ~/robocomp/components/robocomp-robolab/components/viewers/Camera_RGBDSimple_viewer"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 19 "bin/Camera_RGBDSimple_viewer etc/config"






