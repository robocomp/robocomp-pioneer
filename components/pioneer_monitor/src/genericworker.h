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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>

#include <BatteryStatus.h>
#include <CameraSimple.h>
#include <DifferentialRobot.h>
#include <FullPoseEstimation.h>
#include <FullPoseEstimationPub.h>
#include <GenericBase.h>
#include <GpsUblox.h>
#include <JoystickAdapter.h>
#include <Laser.h>
#include <MonitorBase.h>
#include <RSSIStatus.h>
#include <Radar.h>
#include <Ultrasound.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


using TuplePrx = std::tuple<RoboCompBatteryStatus::BatteryStatusPrxPtr,RoboCompCameraSimple::CameraSimplePrxPtr,RoboCompDifferentialRobot::DifferentialRobotPrxPtr,RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr,RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr,RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr,RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr,RoboCompGenericBase::GenericBasePrxPtr,RoboCompGpsUblox::GpsUbloxPrxPtr,RoboCompJoystickAdapter::JoystickAdapterPrxPtr,RoboCompLaser::LaserPrxPtr,RoboCompLaser::LaserPrxPtr,RoboCompLaser::LaserPrxPtr,RoboCompMonitorBase::MonitorBasePrxPtr,RoboCompMonitorBase::MonitorBasePrxPtr,RoboCompMonitorBase::MonitorBasePrxPtr,RoboCompMonitorBase::MonitorBasePrxPtr,RoboCompMonitorBase::MonitorBasePrxPtr,RoboCompMonitorBase::MonitorBasePrxPtr,RoboCompMonitorBase::MonitorBasePrxPtr,RoboCompMonitorBase::MonitorBasePrxPtr,RoboCompRSSIStatus::RSSIStatusPrxPtr,RoboCompRadar::RadarPrxPtr,RoboCompUltrasound::UltrasoundPrxPtr>;


class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	RoboCompBatteryStatus::BatteryStatusPrxPtr batterystatus_proxy;
	RoboCompCameraSimple::CameraSimplePrxPtr camerasimple_proxy;
	RoboCompDifferentialRobot::DifferentialRobotPrxPtr differentialrobot_proxy;
	RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr fullposeestimation_proxy;
	RoboCompFullPoseEstimation::FullPoseEstimationPrxPtr fullposeestimation1_proxy;
	RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr fullposeestimationpub_proxy;
	RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr fullposeestimationpub1_proxy;
	RoboCompGenericBase::GenericBasePrxPtr genericbase_proxy;
	RoboCompGpsUblox::GpsUbloxPrxPtr gpsublox_proxy;
	RoboCompJoystickAdapter::JoystickAdapterPrxPtr joystickadapter_proxy;
	RoboCompLaser::LaserPrxPtr laser_proxy;
	RoboCompLaser::LaserPrxPtr laser1_proxy;
	RoboCompLaser::LaserPrxPtr laser2_proxy;
	RoboCompMonitorBase::MonitorBasePrxPtr monitorbase_proxy;
	RoboCompMonitorBase::MonitorBasePrxPtr monitorbase1_proxy;
	RoboCompMonitorBase::MonitorBasePrxPtr monitorbase2_proxy;
	RoboCompMonitorBase::MonitorBasePrxPtr monitorbase3_proxy;
	RoboCompMonitorBase::MonitorBasePrxPtr monitorbase4_proxy;
	RoboCompMonitorBase::MonitorBasePrxPtr monitorbase5_proxy;
	RoboCompMonitorBase::MonitorBasePrxPtr monitorbase6_proxy;
	RoboCompMonitorBase::MonitorBasePrxPtr monitorbase7_proxy;
	RoboCompRSSIStatus::RSSIStatusPrxPtr rssistatus_proxy;
	RoboCompRadar::RadarPrxPtr radar_proxy;
	RoboCompUltrasound::UltrasoundPrxPtr ultrasound_proxy;

	virtual void DifferentialRobot_correctOdometer(int x, int z, float alpha) = 0;
	virtual void DifferentialRobot_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void DifferentialRobot_resetOdometer() = 0;
	virtual void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state) = 0;
	virtual void DifferentialRobot_setOdometerPose(int x, int z, float alpha) = 0;
	virtual void DifferentialRobot_setSpeedBase(float adv, float rot) = 0;
	virtual void DifferentialRobot_stopBase() = 0;
	virtual void JoystickAdapter_sendData (RoboCompJoystickAdapter::TData data) = 0;

protected:

	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
};

#endif
