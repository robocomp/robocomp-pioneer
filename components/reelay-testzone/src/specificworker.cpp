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
#include <limits.h>
#include "specificworker.h"
#include <reelay/monitors.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
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

	return true;
}

#include <iostream>

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	auto opts = reelay::discrete_timed<intmax_t>
		::monitor<reelay::json, reelay::json>
		::options()
		.disable_condensing()
		.with_time_field_name("timestamp")
		.with_value_field_name("valid");
	auto expr = "true";

	this->monitor = reelay::make_monitor(expr, opts);
	this->Period = period;
	this->min_ldist = std::numeric_limits<float>::infinity();
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
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}

	float threshold = std::numeric_limits<float>::infinity(); // millimeters
	float rot = 0.6;  // rads per second

	try
	{
		// read laser data 
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData(); 
		//sort laser data from small to large distances using a lambda function.
		std::sort(ldata.begin(), ldata.end(), [](auto a, auto b){ return a.dist < b.dist; });  

		if( ldata.front().dist < threshold)
		{
			this->min_ldist = threshold = ldata.front().dist;
			//std::cout << ldata.front().dist << std::endl;
			//differentialrobot_proxy->setSpeedBase(5, rot);
			//usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
		}
		else
		{
			differentialrobot_proxy->setSpeedBase(200, 0); 
		}
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




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
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

