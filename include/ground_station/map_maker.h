#ifndef MAP_MAKERH
#define MAP_MAKERH

#include <QtCore/QObject>
#include <QtCore/QDebug>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "ground_station/drone.h"
#include "ground_station/map.h"

class MapMaker : public QObject {
  Q_OBJECT
public:
  explicit MapMaker(ros::NodeHandle * nh, QObject * parent = 0);
  ~MapMaker();
  
private:
  ros::NodeHandle * _nh;
  
  ros::Publisher _markersPub;
  QMap<std::string, ros::Publisher> _pathPubs;
  
  Map * _map;
  
  void fetchPublishers();
  void fetchMap();
  
  void addNewInfoToMap(Map * map, Drone * drone, const navpts_group::PoseArrayID & posesInfo);
  
signals:
  void signalCorrectedPosesInfo(navpts_group::PoseArrayID posesInfo);

public slots:
  void startMapMaker();
  void correctPosesInfo(Drone * drone, navpts_group::PoseArrayID posesInfo);
  void updateRVizMap(PosesVisualData posesVisualData, navpts_group::PoseArrayID *posesInfo);
};
  
#endif
