#ifndef MAP_MAKERH
#define MAP_MAKERH

#include <QtCore/QObject>
#include <QtCore/QDebug>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "ground_station/drone.h"

#define MAP_TOPIC "map_topic"

class MapMaker : public QObject {
  Q_OBJECT
public:
  explicit MapMaker(ros::NodeHandle * nh, QObject * parent = 0);
  ~MapMaker();
  
private:
  ros::NodeHandle * _nh;
  
  ros::Publisher _visualPub;
  
  QHash<Drone*,visualization_msgs::Marker::_color_type>_drones;
  
  void fetchPublishers();
  void addNewDrone(Drone * drone);
  void updateRVizMap(const std::string ns, const visualization_msgs::Marker::_color_type & color,
		     const geometry_msgs::PoseArray & markerInfo);
  
signals:
  void signalCorrectedMarkerInfo(geometry_msgs::PoseArray markerInfo);

public slots:
  void startMapMaker();
  void correctMarkerInfo(Drone * drone, geometry_msgs::PoseArray markerInfo);
};
  
#endif