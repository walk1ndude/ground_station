#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ground_station/ground_station.h"

#define CAMERA_WINDOW "camera_ardrone"
#define TEST_CAMERA "/drone0/ardrone/image_raw"

GroundStation::GroundStation(QObject * parent) :
  QObject(parent),
  _nhPrivate("~") {
  cv::namedWindow(CAMERA_WINDOW, CV_WINDOW_AUTOSIZE);
  imageSubscriber = _nh.subscribe<sensor_msgs::Image>(TEST_CAMERA, 1, &GroundStation::processImage, this);
  
  fetchParams();
  launchDrones();
}

GroundStation::~GroundStation() {
  qDeleteAll(_drones.begin(), _drones.end());
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

// just for test image processing
void GroundStation::processImage(const sensor_msgs::ImageConstPtr & msg) {
  cv_bridge::CvImagePtr im_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  
  cv::imshow(CAMERA_WINDOW, im_ptr->image);
  cv::waitKey(0);
}
