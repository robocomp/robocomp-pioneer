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
#include "genericworker.h"
/**
* \brief Default constructor
*/
GenericWorker::GenericWorker(TuplePrx tprx) : QObject()
{

	batterystatus_proxy = std::get<0>(tprx);
	camerasimple_proxy = std::get<1>(tprx);
	differentialrobot_proxy = std::get<2>(tprx);
	fullposeestimation_proxy = std::get<3>(tprx);
	fullposeestimation1_proxy = std::get<4>(tprx);
	fullposeestimationpub_proxy = std::get<5>(tprx);
	fullposeestimationpub1_proxy = std::get<6>(tprx);
	genericbase_proxy = std::get<7>(tprx);
	gpsublox_proxy = std::get<8>(tprx);
	joystickadapter_proxy = std::get<9>(tprx);
	laser_proxy = std::get<10>(tprx);
	laser1_proxy = std::get<11>(tprx);
	laser2_proxy = std::get<12>(tprx);
	monitorbase_proxy = std::get<13>(tprx);
	monitorbase1_proxy = std::get<14>(tprx);
	monitorbase2_proxy = std::get<15>(tprx);
	monitorbase3_proxy = std::get<16>(tprx);
	monitorbase4_proxy = std::get<17>(tprx);
	monitorbase5_proxy = std::get<18>(tprx);
	monitorbase6_proxy = std::get<19>(tprx);
	monitorbase7_proxy = std::get<20>(tprx);
	rssistatus_proxy = std::get<21>(tprx);
	radar_proxy = std::get<22>(tprx);
	ultrasound_proxy = std::get<23>(tprx);

	mutex = new QMutex(QMutex::Recursive);

	Period = BASIC_PERIOD;
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));

}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void GenericWorker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
	timer.start(Period);
}
