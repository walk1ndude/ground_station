#ifndef MAP_H
#define MAP_H

#include <QtCore/QObject>
#include <QtCore/QMutex>
#include <QtCore/QDebug>
#include <QtCore/QHash>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/Marker.h>

#include "ground_station/drone.h"

#define MARKER_SIMILARITY_BORDER 0.1 // a sphere around marker to seperate really new or previously found markers

typedef struct _PosesVisualData {
  std::string ns;
  visualization_msgs::Marker::_color_type color;
  visualization_msgs::Marker::_scale_type scale;
  visualization_msgs::Marker::_action_type action;
  visualization_msgs::Marker::_type_type type;
  visualization_msgs::Marker::_lifetime_type lifetime;
  
  _PosesVisualData() {};
  _PosesVisualData(const std::string ns,
		   const visualization_msgs::Marker::_color_type & color = _PosesVisualData::generetaRandomColor(),
		   const visualization_msgs::Marker::_scale_type & scale = _PosesVisualData::defaultScale(),
		   const visualization_msgs::Marker::_action_type & action = visualization_msgs::Marker::MODIFY,
		   const visualization_msgs::Marker::_type_type & type = visualization_msgs::Marker::SPHERE,
		   const visualization_msgs::Marker::_lifetime_type & lifetime = ros::Duration()) :
    ns(ns),
    color(color),
    scale(scale),
    action(action),
    type(type),
    lifetime(lifetime) {};
    
  static visualization_msgs::Marker::_color_type generetaRandomColor() {
    visualization_msgs::Marker::_color_type color;
    
    color.a = 1.0;
    color.b = qrand() / (float) RAND_MAX;
    color.g = qrand() / (float) RAND_MAX;
    color.r = qrand() / (float) RAND_MAX;
    
    return color;
  }
    
  static visualization_msgs::Marker::_scale_type defaultScale() {
    visualization_msgs::Marker::_scale_type scale;
    
    scale.x = 0.1;
    scale.y = 0.1;
    scale.z = 0.1;
    
    return scale;
  }
}PosesVisualData;

class Map : public QObject {
  Q_OBJECT
public:
  explicit Map(const PosesVisualData & posesVisualData, QObject * parent = 0);
  ~Map();
  
  void addNewPoses(Drone * drone, const geometry_msgs::PoseArray & posesInfo);
  
private:
  QHash<Drone*, PosesVisualData>_drones;
  
  QHash<Drone*, QVector<geometry_msgs::PoseStamped> >_posesByDrone;
  QVector<geometry_msgs::PoseStamped>_OverallMap;
  
  PosesVisualData _posesVisualData;
  
  QMutex _mapMutex;
  
  void addNewDroneRViz(Drone * drone);
  void addNewDroneMap(Drone * drone, const geometry_msgs::PoseArray & posesInfo);
  
  void updateDroneMap(Drone * drone, const geometry_msgs::PoseArray & posesInfo);
  
  void updateRViz();
  
  //another solution to create own PoseStampedArray.. but for what reason? is there so much overhead?
  
  static geometry_msgs::PoseArray qVPosesArrayToPoseArray(const QVector<geometry_msgs::PoseStamped> & posesStamped);
  static QVector<geometry_msgs::PoseStamped> posesArrayToQVPoseArray(const geometry_msgs::PoseArray & posesArray);
  
signals:
  void signalUpdateRViz(PosesVisualData posesVisualData, geometry_msgs::PoseArray posesInfo);
};

#endif