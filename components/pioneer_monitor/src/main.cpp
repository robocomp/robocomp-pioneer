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


/** \mainpage RoboComp::pioneer_monitor
 *
 * \section intro_sec Introduction
 *
 * The pioneer_monitor component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd pioneer_monitor
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/pioneer_monitor --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtWidgets>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <differentialrobotI.h>
#include <joystickadapterI.h>




class pioneer_monitor : public RoboComp::Application
{
public:
	pioneer_monitor (QString prfx, bool startup_check) { prefix = prfx.toStdString(); this->startup_check_flag=startup_check; }
private:
	void initialize();
	std::string prefix;
	TuplePrx tprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

void ::pioneer_monitor::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::pioneer_monitor::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

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

	string proxy, tmp;
	initialize();

	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "BatteryStatusProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy BatteryStatusProxy\n";
		}
		batterystatus_proxy = Ice::uncheckedCast<RoboCompBatteryStatus::BatteryStatusPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy BatteryStatus: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("BatteryStatusProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "CameraSimpleProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CameraSimpleProxy\n";
		}
		camerasimple_proxy = Ice::uncheckedCast<RoboCompCameraSimple::CameraSimplePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy CameraSimple: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("CameraSimpleProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "DifferentialRobotProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy DifferentialRobotProxy\n";
		}
		differentialrobot_proxy = Ice::uncheckedCast<RoboCompDifferentialRobot::DifferentialRobotPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy DifferentialRobot: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("DifferentialRobotProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "FullPoseEstimationProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy FullPoseEstimationProxy\n";
		}
		fullposeestimation_proxy = Ice::uncheckedCast<RoboCompFullPoseEstimation::FullPoseEstimationPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy FullPoseEstimation: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("FullPoseEstimationProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "FullPoseEstimation1Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy FullPoseEstimationProxy\n";
		}
		fullposeestimation1_proxy = Ice::uncheckedCast<RoboCompFullPoseEstimation::FullPoseEstimationPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy FullPoseEstimation1: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("FullPoseEstimationProxy1 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "FullPoseEstimationPubProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy FullPoseEstimationPubProxy\n";
		}
		fullposeestimationpub_proxy = Ice::uncheckedCast<RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy FullPoseEstimationPub: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("FullPoseEstimationPubProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "FullPoseEstimationPub1Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy FullPoseEstimationPubProxy\n";
		}
		fullposeestimationpub1_proxy = Ice::uncheckedCast<RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy FullPoseEstimationPub1: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("FullPoseEstimationPubProxy1 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "GenericBaseProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy GenericBaseProxy\n";
		}
		genericbase_proxy = Ice::uncheckedCast<RoboCompGenericBase::GenericBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy GenericBase: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("GenericBaseProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "GpsUbloxProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy GpsUbloxProxy\n";
		}
		gpsublox_proxy = Ice::uncheckedCast<RoboCompGpsUblox::GpsUbloxPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy GpsUblox: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("GpsUbloxProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "JoystickAdapterProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy JoystickAdapterProxy\n";
		}
		joystickadapter_proxy = Ice::uncheckedCast<RoboCompJoystickAdapter::JoystickAdapterPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy JoystickAdapter: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("JoystickAdapterProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "LaserProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy LaserProxy\n";
		}
		laser_proxy = Ice::uncheckedCast<RoboCompLaser::LaserPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy Laser: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("LaserProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "Laser1Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy LaserProxy\n";
		}
		laser1_proxy = Ice::uncheckedCast<RoboCompLaser::LaserPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy Laser1: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("LaserProxy1 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "Laser2Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy LaserProxy\n";
		}
		laser2_proxy = Ice::uncheckedCast<RoboCompLaser::LaserPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy Laser2: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("LaserProxy2 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "MonitorBaseProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy MonitorBaseProxy\n";
		}
		monitorbase_proxy = Ice::uncheckedCast<RoboCompMonitorBase::MonitorBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy MonitorBase: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("MonitorBaseProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "MonitorBase1Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy MonitorBaseProxy\n";
		}
		monitorbase1_proxy = Ice::uncheckedCast<RoboCompMonitorBase::MonitorBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy MonitorBase1: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("MonitorBaseProxy1 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "MonitorBase2Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy MonitorBaseProxy\n";
		}
		monitorbase2_proxy = Ice::uncheckedCast<RoboCompMonitorBase::MonitorBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy MonitorBase2: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("MonitorBaseProxy2 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "MonitorBase3Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy MonitorBaseProxy\n";
		}
		monitorbase3_proxy = Ice::uncheckedCast<RoboCompMonitorBase::MonitorBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy MonitorBase3: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("MonitorBaseProxy3 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "MonitorBase4Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy MonitorBaseProxy\n";
		}
		monitorbase4_proxy = Ice::uncheckedCast<RoboCompMonitorBase::MonitorBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy MonitorBase4: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("MonitorBaseProxy4 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "MonitorBase5Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy MonitorBaseProxy\n";
		}
		monitorbase5_proxy = Ice::uncheckedCast<RoboCompMonitorBase::MonitorBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy MonitorBase5: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("MonitorBaseProxy5 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "MonitorBase6Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy MonitorBaseProxy\n";
		}
		monitorbase6_proxy = Ice::uncheckedCast<RoboCompMonitorBase::MonitorBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy MonitorBase6: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("MonitorBaseProxy6 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "MonitorBase7Proxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy MonitorBaseProxy\n";
		}
		monitorbase7_proxy = Ice::uncheckedCast<RoboCompMonitorBase::MonitorBasePrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy MonitorBase7: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("MonitorBaseProxy7 initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "RSSIStatusProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy RSSIStatusProxy\n";
		}
		rssistatus_proxy = Ice::uncheckedCast<RoboCompRSSIStatus::RSSIStatusPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy RSSIStatus: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("RSSIStatusProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "RadarProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy RadarProxy\n";
		}
		radar_proxy = Ice::uncheckedCast<RoboCompRadar::RadarPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy Radar: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("RadarProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "UltrasoundProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy UltrasoundProxy\n";
		}
		ultrasound_proxy = Ice::uncheckedCast<RoboCompUltrasound::UltrasoundPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy Ultrasound: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("UltrasoundProxy initialized Ok!");


	IceStorm::TopicManagerPrxPtr topicManager;
	try
	{
		topicManager = topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->propertyToProxy("TopicManager.Proxy"));
		if (!topicManager)
		{
		    cout << "[" << PROGRAM_NAME << "]: TopicManager.Proxy not defined in config file."<<endl;
		    cout << "	 Config line example: TopicManager.Proxy=IceStorm/TopicManager:default -p 9999"<<endl;
	        return EXIT_FAILURE;
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: 'rcnode' not running: " << ex << endl;
		return EXIT_FAILURE;
	}

	tprx = std::make_tuple(batterystatus_proxy,camerasimple_proxy,differentialrobot_proxy,fullposeestimation_proxy,fullposeestimation1_proxy,fullposeestimationpub_proxy,fullposeestimationpub1_proxy,genericbase_proxy,gpsublox_proxy,joystickadapter_proxy,laser_proxy,laser1_proxy,laser2_proxy,monitorbase_proxy,monitorbase1_proxy,monitorbase2_proxy,monitorbase3_proxy,monitorbase4_proxy,monitorbase5_proxy,monitorbase6_proxy,monitorbase7_proxy,rssistatus_proxy,radar_proxy,ultrasound_proxy);
	SpecificWorker *worker = new SpecificWorker(tprx, startup_check_flag);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}



		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "DifferentialRobot.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy DifferentialRobot";
			}
			Ice::ObjectAdapterPtr adapterDifferentialRobot = communicator()->createObjectAdapterWithEndpoints("DifferentialRobot", tmp);
			auto differentialrobot = std::make_shared<DifferentialRobotI>(worker);
			adapterDifferentialRobot->add(differentialrobot, Ice::stringToIdentity("differentialrobot"));
			adapterDifferentialRobot->activate();
			cout << "[" << PROGRAM_NAME << "]: DifferentialRobot adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for DifferentialRobot\n";
		}


		// Server adapter creation and publication
		std::shared_ptr<IceStorm::TopicPrx> joystickadapter_topic;
		Ice::ObjectPrxPtr joystickadapter;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "JoystickAdapterTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy JoystickAdapterProxy";
			}
			Ice::ObjectAdapterPtr JoystickAdapter_adapter = communicator()->createObjectAdapterWithEndpoints("joystickadapter", tmp);
			RoboCompJoystickAdapter::JoystickAdapterPtr joystickadapterI_ =  std::make_shared <JoystickAdapterI>(worker);
			auto joystickadapter = JoystickAdapter_adapter->addWithUUID(joystickadapterI_)->ice_oneway();
			if(!joystickadapter_topic)
			{
				try {
					joystickadapter_topic = topicManager->create("JoystickAdapter");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						joystickadapter_topic = topicManager->retrieve("JoystickAdapter");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				catch(const IceUtil::NullHandleException&)
				{
					cout << "[" << PROGRAM_NAME << "]: ERROR TopicManager is Null. Check that your configuration file contains an entry like:\n"<<
					"\t\tTopicManager.Proxy=IceStorm/TopicManager:default -p <port>\n";
					return EXIT_FAILURE;
				}
				IceStorm::QoS qos;
				joystickadapter_topic->subscribeAndGetPublisher(qos, joystickadapter);
			}
			JoystickAdapter_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating JoystickAdapter topic.\n";
			//Error. Topic does not exist
		}


		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();

		try
		{
			std::cout << "Unsubscribing topic: joystickadapter " <<std::endl;
			joystickadapter_topic->unsubscribe( joystickadapter );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: joystickadapter " << ex.what()<<std::endl;
		}


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	QString configFile("etc/config");
	bool startup_check_flag = false;
	QString prefix("");
	if (argc > 1)
	{

		// Search in argument list for arguments
		QString startup = QString("--startup-check");
		QString initIC = QString("--Ice.Config=");
		QString prfx = QString("--prefix=");
		for (int i = 0; i < argc; ++i)
		{
			arg = argv[i];
			if (arg.find(startup.toStdString(), 0) != std::string::npos)
			{
				startup_check_flag = true;
				cout << "Startup check = True"<< endl;
			}
			else if (arg.find(prfx.toStdString(), 0) != std::string::npos)
			{
				prefix = QString::fromStdString(arg).remove(0, prfx.size());
				if (prefix.size()>0)
					prefix += QString(".");
				printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
			}
			else if (arg.find(initIC.toStdString(), 0) != std::string::npos)
			{
				configFile = QString::fromStdString(arg).remove(0, initIC.size());
				qDebug()<<__LINE__<<"Starting with config file:"<<configFile;
			}
			else if (i==1 and argc==2 and arg.find("--", 0) == std::string::npos)
			{
				configFile = QString::fromStdString(arg);
				qDebug()<<__LINE__<<QString::fromStdString(arg)<<argc<<arg.find("--", 0)<<"Starting with config file:"<<configFile;
			}
		}

	}
	::pioneer_monitor app(prefix, startup_check_flag);

	return app.main(argc, argv, configFile.toLocal8Bit().data());
}
