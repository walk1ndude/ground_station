#ifndef MAP_MAKERH
#define MAP_MAKERH

#include <QtCore/QObject>
#include <QtCore/QDebug>

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include "ground_station/drone.h"
#include "ground_station/map.h"

#define MAP_TOPIC "map_topic"

class MapMaker : public QObject {
  Q_OBJECT
public:
  explicit MapMaker(ros::NodeHandle * nh, QObject * parent = 0);
  ~MapMaker();
  
private:
  ros::NodeHandle * _nh;
  
  ros::Publisher _visualPub;
  
  Map * _map;
  
  void fetchPublishers();
  void fetchMap();
  
  void addNewInfoToMap(Map * map, Drone * drone, const navpts::PoseArrayID & posesInfo);
  
signals:
  void signalCorrectedPosesInfo(navpts::PoseArrayID posesInfo);

public slots:
  void startMapMaker();
  void correctPosesInfo(Drone * drone, navpts::PoseArrayID posesInfo);
  void updateRVizMap(PosesVisualData posesVisualData, geometry_msgs::PoseArray posesInfo);
};
  
#endif