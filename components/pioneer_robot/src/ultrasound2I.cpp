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
#include "ultrasound2I.h"

Ultrasound2I::Ultrasound2I(GenericWorker *_worker)
{
	worker = _worker;
}


Ultrasound2I::~Ultrasound2I()
{
}


RoboCompUltrasound2::SensorsState Ultrasound2I::getAllSensorDistances(const Ice::Current&)
{
	return worker->Ultrasound2_getAllSensorDistances();
}

RoboCompUltrasound2::SensorParamsList Ultrasound2I::getAllSensorParams(const Ice::Current&)
{
	return worker->Ultrasound2_getAllSensorParams();
}

RoboCompUltrasound2::SonarPoseList Ultrasound2I::getAllSonarPose(const Ice::Current&)
{
	return worker->Ultrasound2_getAllSonarPose();
}

RoboCompUltrasound2::BusParams Ultrasound2I::getBusParams(const Ice::Current&)
{
	return worker->Ultrasound2_getBusParams();
}

int Ultrasound2I::getSensorDistance(std::string sensor, const Ice::Current&)
{
	return worker->Ultrasound2_getSensorDistance(sensor);
}

RoboCompUltrasound2::SensorParams Ultrasound2I::getSensorParams(std::string sensor, const Ice::Current&)
{
	return worker->Ultrasound2_getSensorParams(sensor);
}

int Ultrasound2I::getSonarsNumber(const Ice::Current&)
{
	return worker->Ultrasound2_getSonarsNumber();
}

