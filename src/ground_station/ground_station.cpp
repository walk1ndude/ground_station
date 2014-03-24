#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ground_station/ground_station.h"
#include <../../tum_ardrone/src/stateestimation/PTAM/Map.h>

#define CAMERA_WINDOW "camera_ardrone"
#define TEST_CAMERA "/ardrone/image_raw"

GroundStation::GroundStation(QObject * parent) :
  QObject(parent),
  _nhPrivate("~") {
  
  fetchMapMaker();
  fetchParams();
  launchDrones();
}

GroundStation::~GroundStation() {
  qDeleteAll(_drones.begin(), _drones.end());
  delete _mapMaker;
}

void GroundStation::loop(const int & frequency) {
  ros::Rate sleep_rate(frequency);
  
  while(_nh.ok()) {
    ros::spinOnce();
    
    sleep_rate.sleep();
  }
}

void GroundStation::fetchMapMaker() {
  _mapMaker = new MapMaker;
  QThread * mapThread = new QThread;
  _mapMaker->moveToThread(mapThread);
  
  QObject::connect(_mapMaker, &MapMaker::destroyed, mapThread, &QThread::quit);
  QObject::connect(mapThread, &QThread::finished, mapThread, &QThread::deleteLater);
  
  mapThread->start();
}

void GroundStation::fetchParams() {
  std::vector<std::string>droneDrivers;
  XmlRpc::XmlRpcValue drivers;
  
  _nhPrivate.getParam("droneDrivers", drivers);
  ROS_ASSERT(drivers.getType() == XmlRpc::XmlRpcValue::TypeArray);
      
  for (int i = 0; i < drivers.size(); i ++) {
    droneDrivers.push_back(static_cast<std::string>(drivers[i]));
  }
  
  fetchDrones(droneDrivers);
}

void GroundStation::fetchDrones(const std::vector<std::string> & droneDrivers) {
  DroneData droneData;
  
  for (int i = 0; i != droneDrivers.size(); ++ i) {

    QThread * thread = new QThread;
    
    droneData.id = i;
    droneData.driver = droneDrivers[i];
    
    Drone * drone = new Drone(droneData, &_nh);
    drone->moveToThread(thread);
    
    QObject::connect(thread, &QThread::started, drone, &Drone::startTask);
    QObject::connect(drone, (void (Drone::*)(Drone*))&Drone::signalTaskFinished,
		  this, (void (GroundStation::*)(Drone*))&GroundStation::removeDrone, Qt::DirectConnection);
    QObject::connect(drone, (void (Drone::*)(geometry_msgs::PoseArray))&Drone::signalCorrectMarkerInfo,
		   _mapMaker, (void (MapMaker::*)(geometry_msgs::PoseArray))&MapMaker::correctMarkerInfo, Qt::DirectConnection);
    QObject::connect(_mapMaker, (void (MapMaker::*)(geometry_msgs::PoseArray))&MapMaker::signalCorrectedMarkerInfo,
		     drone, (void (Drone::*)(geometry_msgs::PoseArray))&Drone::setMarkerInfo, Qt::DirectConnection);
    QObject::connect(drone, &Drone::destroyed, thread, &QThread::quit);
    QObject::connect(thread, &QThread::finished, thread, &QThread::deleteLater);
    
    _drones.insert(drone);
    
    qDebug() << "drone" << i << " fetched";
  }
}

void GroundStation::launchDrones() {
  QSetIterator<Drone*>it(_drones);
  
  while(it.hasNext()) {
    it.next()->thread()->start();
  }
}

void GroundStation::removeDrone(Drone * drone) {
  _drones.remove(drone);
  drone->deleteLater();
}
