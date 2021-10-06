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
#ifndef ULTRASOUND2_H
#define ULTRASOUND2_H

// Ice includes
#include <Ice/Ice.h>
#include <Ultrasound2.h>

#include <config.h>
#include "genericworker.h"


class Ultrasound2I : public virtual RoboCompUltrasound2::Ultrasound2
{
public:
	Ultrasound2I(GenericWorker *_worker);
	~Ultrasound2I();

	RoboCompUltrasound2::SensorsState getAllSensorDistances(const Ice::Current&);
	RoboCompUltrasound2::SensorParamsList getAllSensorParams(const Ice::Current&);
	RoboCompUltrasound2::SonarPoseList getAllSonarPose(const Ice::Current&);
	RoboCompUltrasound2::BusParams getBusParams(const Ice::Current&);
	int getSensorDistance(std::string sensor, const Ice::Current&);
	RoboCompUltrasound2::SensorParams getSensorParams(std::string sensor, const Ice::Current&);
	int getSonarsNumber(const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
