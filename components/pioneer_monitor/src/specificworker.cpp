/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include "cppitertools/sliding_window.hpp"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    bumper = (params["bumper"].value == "true") or (params["bumper"].value == "True");
    std::cout<<params["bumper"].value<<std::endl;
    std::cout<<bumper<<std::endl;

    //bool bumper = false;
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

//RoboCompJoystickAdapter::TData joydata;

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
   
    //auto ldata = laser_proxy->getLaserData();
    auto ldata_front = laser_proxy->getLaserData();
    auto ldata_back = laser1_proxy->getLaserData();

    RoboCompUltrasound::SensorsState sonar;
    sonar = this->ultrasound_proxy->getAllSensorDistances();
    std::vector<float> frontal_distances;
    std::vector<float> reverse_distances;
    int stop_threshold = 600, slow_threshold = 2000, slow_mid_threshold=1200, min_speed = 0, max_speed = 1000, residue = 200, trim = 3; 
    float m = (max_speed - min_speed) * 1. / (slow_threshold - stop_threshold);
    float n = min_speed - m * stop_threshold + residue;
    float adv_speed = 0;
    float adv_speed_sonar=0, adv_speed_laser=0, rot_speed_sonar, rot_speed_laser;
    float rot_speed = 0;
    bool  atras = false;
    float reverse_speed = 0;


    //Ping to DSR machine

    int x = system("ping -c1 -s1 melex.local  > /dev/null 2>&1");
    if (x==0){
        cout<<"success"<<endl;
    }else{
        cout<<"ping failed"<<endl;
        joydata.axes[1].value=0;
        joydata.axes[0].value=0;
    }


    for(auto  &a: joydata.axes)
    {
        if(a.name == "advance"){
            adv_speed = std::clamp(a.value, -1000.f, 1000.f);
            //std::cout << "M y N" << std::endl;
            float m = (adv_speed - min_speed) * 1. / (slow_threshold - stop_threshold);
            //std::cout << m << std:: endl;
            float n = min_speed - m * stop_threshold + residue;
            //std::cout << n << std:: endl;
        }
        if(a.name == "turn"){
            rot_speed = std::clamp(a.value, -100.f, 100.f);
            //std::cout << "Rotation: << std::endl;
        }

    }

    if(fabs(rot_speed) < 2) rot_speed = 0;
    if(fabs(adv_speed) < 40) adv_speed = 0;

    float rot = qDegreesToRadians(rot_speed);
    if (bumper){
        //LASER_BUMPER
        if (adv_speed>=0){
            std::vector<float> distances_front_laser;
            for(auto &point_front_laser : ldata_front)
                distances_front_laser.push_back(point_front_laser.dist);
            if(distances_front_laser[0] < 200)
                distances_front_laser[0] = 200;
            for(auto &&window_front_laser : iter::sliding_window(distances_front_laser, 2))
            {
                if(window_front_laser[1] < 200)
                    window_front_laser[1] = window_front_laser[0];
            }
            int limit = distances_front_laser.size()/trim;
            std::sort(distances_front_laser.begin() + limit, distances_front_laser.end() - limit, [](float a, float b){return a < b;});
            float minValue_front_laser = distances_front_laser[limit];
            if (minValue_front_laser< stop_threshold){
                adv_speed_laser = 0;
                rot_speed_laser = 0;
            }else if(minValue_front_laser< slow_threshold){
                adv_speed_laser = adv_speed*0.40;
                
            }else if(minValue_front_laser< slow_mid_threshold ){
                adv_speed_laser = adv_speed*0.1;
                }
            
            else
                adv_speed_laser = adv_speed;

        }else{
            std::vector<float> distances_back_laser;
            for(auto &point_back_laser : ldata_back)
                distances_back_laser.push_back(point_back_laser.dist);
            if(distances_back_laser[0] < 200)
                distances_back_laser[0] = 200;
            for(auto &&window_back_laser : iter::sliding_window(distances_back_laser, 2))
            {
                if(window_back_laser[1] < 200)
                    window_back_laser[1] = window_back_laser[0];
            }
            int limit = distances_back_laser.size()/trim;
            std::sort(distances_back_laser.begin() + limit, distances_back_laser.end() - limit, [](float a, float b){return a < b;});
            float minValue_back_laser = distances_back_laser[limit];
            if (minValue_back_laser< stop_threshold){
                adv_speed_laser = 0;
                rot_speed_laser = 0;
            }else if(minValue_back_laser< slow_threshold){
                adv_speed_laser = adv_speed*0.40;
                
            }else if(minValue_back_laser< slow_mid_threshold ){
                adv_speed_laser = adv_speed*0.1;
                }
            
            else
                adv_speed_laser = adv_speed;

        }    

        // BUMPER SONAR
        
        if (adv_speed >0){
            for (int i = 2; i < 6; i++){
            frontal_distances.push_back(sonar[i]);
            }
            std::sort(frontal_distances.begin(), frontal_distances.end(), [](float a, float b){return a < b;});
            float min_front_dist=frontal_distances[0];
            if (min_front_dist < stop_threshold){
                adv_speed_sonar = 0;
                rot_speed_sonar = 0;
            }else if(min_front_dist < slow_threshold){
                adv_speed_sonar = adv_speed*0.40;
                
            }else if(min_front_dist < slow_mid_threshold ){
                adv_speed_sonar = adv_speed*0.1;
                }
            
            else
                adv_speed_sonar = adv_speed;
        }else{
                for (int i = 10; i < 14; i++){
                reverse_distances.push_back(sonar[i]);
            }
            
                std::sort(reverse_distances.begin(), reverse_distances.end(), [](float a, float b){return a < b;});
                float min_reverse_dist=reverse_distances[0];
                if (min_reverse_dist < stop_threshold){
                adv_speed_sonar = 0;
                rot = 0;
            }else if(min_reverse_dist < slow_threshold){
                adv_speed_sonar = adv_speed*0.40;
                
            }else if( min_reverse_dist < slow_mid_threshold){
                adv_speed_sonar = adv_speed*0.1;
                }
            
            else
                adv_speed_sonar = adv_speed;

        }
        if (adv_speed_laser == adv_speed_sonar and adv_speed_laser == adv_speed){
            cout << "SIN LIMITE";
        }
        else if (abs(adv_speed_sonar)<= abs(adv_speed_laser)){
            adv_speed=adv_speed_sonar;
            cout << "LIMITACION_SONAR";
        }else{
            adv_speed=adv_speed_laser;
            cout << "LIMITACION_LASER";
        }
    }

    
    
    try{
        this->differentialrobot_proxy->setSpeedBase(adv_speed, rot);
        std::cout << "SetSpeed_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "SetSpeed_OFF" << std::endl;}


    try{
        this->monitorbase_proxy->ice_ping();
        std::cout << "BASE_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "Base_off" << std::endl;}
    try{
        this->monitorbase1_proxy->ice_ping();
        std::cout << "GPS_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "GPS_off" << std::endl;

    }
    try{
        this->monitorbase2_proxy->ice_ping();
        std::cout << "LASERF_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "LASERF_off" << std::endl;}
    try{
        this->monitorbase3_proxy->ice_ping();
        std::cout << "LASERB_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "LASERB_off" << std::endl;

    }
    try{
        this->monitorbase4_proxy->ice_ping();
        std::cout << "LASERI_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "LASERI_off" << std::endl;}
    try{
        this->monitorbase5_proxy->ice_ping();
        std::cout << "POSE_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "POSE_off" << std::endl;

    }
    try{
        this->monitorbase6_proxy->ice_ping();
        std::cout << "RADAR_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "RADAR_off" << std::endl;}
    try{
        this->monitorbase7_proxy->ice_ping();
        std::cout << "CAMERA_OK" << std::endl;
    }catch(const Ice::Exception &e) { std::cout << "CAMERA_off" << std::endl;

    }
    //this->monitorbase_proxy->getMonitorState();
    //this->monitorbase1_proxy->getMonitorState();

	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


void SpecificWorker::DifferentialRobot_correctOdometer(int x, int z, float alpha)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_getBasePose(int &x, int &z, float &alpha)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_resetOdometer()
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_setOdometerPose(int x, int z, float alpha)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_setSpeedBase(float adv, float rot)
{
    this->differentialrobot_proxy->setSpeedBase(adv, rot);
}

void SpecificWorker::DifferentialRobot_stopBase()
{
//implementCODE

}

//SUBSCRIPTION to sendData method from JoystickAdapter interface
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    //int stop_threshold = 700, slow_threshold = 1400, min_speed = 0, max_speed = 1000, residue = 200, trim = 3; //COPPELIA VALUES
    joydata = data;
    /* float m = (max_speed - min_speed) * 1. / (slow_threshold - stop_threshold);
    float n = min_speed - m * stop_threshold + residue;
    std::vector<float> frontal;
    RoboCompUltrasound::SensorsState sonar;
    float adv_speed = 0;
    float rot_speed = 0;
    bool  atras = false;
    float reverse_speed = 0;
    for(auto  &a: data.axes)
    {
        if(a.name == "advance"){
            adv_speed = std::clamp(a.value, -1000.f, 1000.f);
            if (adv_speed <0){
                atras = true;
                reverse_speed = adv_speed;
            }
            //std::cout << "M y N" << std::endl;
            //float m = (adv_speed - min_speed) * 1. / (slow_threshold - stop_threshold);
            //std::cout << m << std:: endl;
            //float n = min_speed - m * stop_threshold + residue;
            //std::cout << n << std:: endl;
        }
        if(a.name == "turn"){
            rot_speed = std::clamp(a.value, -100.f, 100.f);
            //std::cout << "Rotation: << std::endl;
        }

    }

    if(fabs(rot_speed) < 1) rot_speed = 0;
    if(fabs(adv_speed) < 40) adv_speed = 0;

    float rot = qDegreesToRadians(rot_speed);
	//cout<< rot;
	//cout<< rot_speed;
    sonar = this->ultrasound_proxy->getAllSensorDistances();
    //int j = 1;
    for (int i = 3; i < 5; i++){
        //frontal[j] = sonar[i];
        //j++;
        if (sonar[i] < stop_threshold){
            adv_speed = 0;
            rot = 0;
            if (atras)
                adv_speed = reverse_speed;
        }else if(sonar[i] < slow_threshold){
            adv_speed = m * sonar[i]+ n;
            std::cout << "DISTANCIA Y AVANCE" << std:: endl;
            std::cout << sonar[i] << std:: endl;
            std::cout << adv_speed << std:: endl;
        }
        else
            adv_speed = adv_speed;
        //else{
           // this->differentialrobot_proxy->setSpeedBase(adv_speed, rot);
        //}
    }
    std::cout << frontal[1] << std:: endl;
    this->differentialrobot_proxy->setSpeedBase(adv_speed, rot);
 */
	 


}



/**************************************/
// From the RoboCompBatteryStatus you can call this methods:
// this->batterystatus_proxy->getBatteryState(...)

/**************************************/
// From the RoboCompBatteryStatus you can use this types:
// RoboCompBatteryStatus::TBattery

/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation_proxy->getFullPoseEuler(...)
// this->fullposeestimation_proxy->getFullPoseMatrix(...)
// this->fullposeestimation_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation1_proxy->getFullPoseEuler(...)
// this->fullposeestimation1_proxy->getFullPoseMatrix(...)
// this->fullposeestimation1_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

/**************************************/
// From the RoboCompFullPoseEstimationPub you can call this methods:
// this->fullposeestimationpub_proxy->newFullPose(...)

/**************************************/
// From the RoboCompFullPoseEstimationPub you can call this methods:
// this->fullposeestimationpub1_proxy->newFullPose(...)

/**************************************/
// From the RoboCompGenericBase you can call this methods:
// this->genericbase_proxy->getBasePose(...)
// this->genericbase_proxy->getBaseState(...)

/**************************************/
// From the RoboCompGenericBase you can use this types:
// RoboCompGenericBase::TBaseState

/**************************************/
// From the RoboCompGpsUblox you can call this methods:
// this->gpsublox_proxy->getData(...)
// this->gpsublox_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompGpsUblox you can use this types:
// RoboCompGpsUblox::DatosGPS

/**************************************/
// From the RoboCompJoystickAdapter you can call this methods:
// this->joystickadapter_proxy->sendData(...)

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser1_proxy->getLaserAndBStateData(...)
// this->laser1_proxy->getLaserConfData(...)
// this->laser1_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser2_proxy->getLaserAndBStateData(...)
// this->laser2_proxy->getLaserConfData(...)
// this->laser2_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompMonitorBase you can call this methods:
// this->monitorbase_proxy->getMonitorState(...)

/**************************************/
// From the RoboCompMonitorBase you can call this methods:
// this->monitorbase1_proxy->getMonitorState(...)

/**************************************/
// From the RoboCompMonitorBase you can call this methods:
// this->monitorbase2_proxy->getMonitorState(...)

/**************************************/
// From the RoboCompMonitorBase you can call this methods:
// this->monitorbase3_proxy->getMonitorState(...)

/**************************************/
// From the RoboCompMonitorBase you can call this methods:
// this->monitorbase4_proxy->getMonitorState(...)

/**************************************/
// From the RoboCompMonitorBase you can call this methods:
// this->monitorbase5_proxy->getMonitorState(...)

/**************************************/
// From the RoboCompMonitorBase you can call this methods:
// this->monitorbase6_proxy->getMonitorState(...)

/**************************************/
// From the RoboCompMonitorBase you can call this methods:
// this->monitorbase7_proxy->getMonitorState(...)

/**************************************/
// From the RoboCompRSSIStatus you can call this methods:
// this->rssistatus_proxy->getRSSIState(...)

/**************************************/
// From the RoboCompRSSIStatus you can use this types:
// RoboCompRSSIStatus::TRSSI

/**************************************/
// From the RoboCompRadar you can call this methods:
// this->radar_proxy->getData(...)

/**************************************/
// From the RoboCompRadar you can use this types:
// RoboCompRadar::RadarData

/**************************************/
// From the RoboCompUltrasound you can call this methods:
// this->ultrasound_proxy->getAllSensorDistances(...)
// this->ultrasound_proxy->getAllSensorParams(...)
// this->ultrasound_proxy->getAllSonarPose(...)
// this->ultrasound_proxy->getBusParams(...)
// this->ultrasound_proxy->getSensorDistance(...)
// this->ultrasound_proxy->getSensorParams(...)
// this->ultrasound_proxy->getSonarsNumber(...)

/**************************************/
// From the RoboCompUltrasound you can use this types:
// RoboCompUltrasound::BusParams
// RoboCompUltrasound::SensorParams
// RoboCompUltrasound::SonarPose

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

