#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ground_station/ground_station.h"

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
  _mapMaker = new MapMaker(&_nh);
  QThread * mapThread = new QThread;
  _mapMaker->moveToThread(mapThread);
  
  QObject::connect(mapThread, &QThread::started, _mapMaker, &MapMaker::startMapMaker);
  QObject::connect(mapThread, &QThread::finished, mapThread, &QThread::deleteLater);
  
  QObject::connect(_mapMaker, &MapMaker::destroyed, mapThread, &QThread::quit);
  
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
  for (int i = 0; i != droneDrivers.size(); ++ i) {

    QThread * thread = new QThread;
    
    std::stringstream stream;
    stream << "drone" << i;
    
    Drone * drone = new Drone(DroneData(i, stream.str(), droneDrivers[i]), &_nh);
    
    drone->moveToThread(thread);
    
    QObject::connect(thread, &QThread::started, drone, &Drone::startTask);
    QObject::connect(drone, &Drone::signalTaskFinished, this, &GroundStation::removeDrone, Qt::DirectConnection);
    QObject::connect(drone, &Drone::signalCorrectPosesInfo, _mapMaker, &MapMaker::correctPosesInfo, Qt::DirectConnection);
    QObject::connect(drone, &Drone::destroyed, thread, &QThread::quit);
    
    QObject::connect(thread, &QThread::finished, thread, &QThread::deleteLater);
    
    QObject::connect(_mapMaker, &MapMaker::signalCorrectedPosesInfo, drone, &Drone::setPosesInfo, Qt::DirectConnection);
    
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
