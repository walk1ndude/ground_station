#ifndef MAP_H
#define MAP_H

#include <QtCore/QObject>
#include <QtCore/QMutex>
#include <QtCore/QDebug>
#include <QtCore/QHash>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/Marker.h>

/*
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
*/
#include <opencv2/core/core.hpp>

#include "ground_station/drone.h"

#define POSE_SIMILAR_TOLERANCE_BORDER 0.5 // a sphere around marker to seperate really new or previously found markers

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
  explicit Map(const PosesVisualData & worldMapVisualData, QObject * parent = 0);
  ~Map();
  
  void addNewPoses(Drone * drone, const navpts::PoseArrayID & posesInfo);
  
private:
  QHash<Drone*, PosesVisualData>_drones;
  
  //QHash<Drone*, navpts::PoseArrayID>_posesByDrone;
  //QHash<Drone*, pcl::PointCloud<pcl::PointXYZ> >_pointClouds;
  QHash<Drone*, QHash<int, geometry_msgs::PoseStamped> >_posesByDrone;
  QHash<int, geometry_msgs::PoseStamped>_worldMap;
  
  QHash<Drone*, cv::Matx44f>_dronesRT;
  
  PosesVisualData _worldMapVisualData;
  
  QMutex _mapMutex;
  
  //for now first drone - map coordinate system's center
  Drone * _droneWorldMap;
  
  void addNewDroneRViz(Drone * drone);
  void addNewDroneMap(Drone * drone, const navpts::PoseArrayID & posesInfo);
  void addNewDroneRT(Drone * drone);
  
  static QHash<int, geometry_msgs::PoseStamped> poseArrayToHash(const navpts::PoseArrayID & posesInfo);
  static geometry_msgs::PoseArray hashToPoseArray(const QHash<int, geometry_msgs::PoseStamped> & posesInfo);
  
  void updateDroneMap(Drone * drone, const navpts::PoseArrayID & posesInfo);
  
  void tryToUpdateWorldMap();
  QVector<Drone*> findDronesWithSimilarPoses(Drone * pivotDrone);
  void findRTMatrices(Drone * pivotDrone, const QVector<Drone*> & dronesWithSimilarPoses);
  
  static std::vector<cv::Point3f> posesToCvPoints(const QHash<int, geometry_msgs::PoseStamped> & poses);
  
  void updateRViz();
  
signals:
  void signalUpdateRViz(PosesVisualData posesVisualData, geometry_msgs::PoseArray posesInfo);
};

#endif