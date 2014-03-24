#ifndef MAP_MAKERH
#define MAP_MAKERH

#include <QtCore/QObject>
#include <QtCore/QDebug>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

class MapMaker : public QObject {
  Q_OBJECT
public:
  explicit MapMaker(QObject * parent = 0);
  ~MapMaker();

private:
  
public slots:
  void correctMarkerInfo(geometry_msgs::PoseArray markerInfo);
};
  
#endif