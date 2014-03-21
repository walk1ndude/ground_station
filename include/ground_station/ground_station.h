#ifndef GROUND_STATIONH
#define GROUND_STATIONH

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <QtCore/QThread>
#include <QtCore/QSet>

#include "ground_station/drone.h"

class GroundStation : public QObject {
  Q_OBJECT
public:
  explicit GroundStation(QObject * parent = 0);
  ~GroundStation();
  
private:
  ros::NodeHandle _nh;
  ros::NodeHandle _nhPrivate;
  
  ros::Subscriber imageSubscriber;
  
  QSet<Drone*>_drones;
  
  void fetchParams();
  
  void inline fetchDrones(const std::vector<std::string> & droneDrivers);
  
  void processImage(const sensor_msgs::ImageConstPtr & msg);

public slots:
  void launchDrones();
  void removeDrone(Drone * drone);
};

#endif