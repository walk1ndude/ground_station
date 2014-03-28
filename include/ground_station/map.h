#ifndef MAP_H
#define MAP_H

#include <QtCore/QObject>
#include <QtCore/QMutex>
#include <QtCore/QDebug>
#include <QtCore/QMap>

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

#define DRONEPOSE_SEQ_ID 1000

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
    color.b = qrand() / (float) RAND_MAX * 100;
    color.g = qrand() / (float) RAND_MAX * 100;
    color.r = qrand() / (float) RAND_MAX * 100;
    
    return color;
  }
    
  static visualization_msgs::Marker::_scale_type defaultScale() {
    visualization_msgs::Marker::_scale_type scale;
    
    scale.x = 0.05;
    scale.y = 0.05;
    scale.z = 0.05;
    
    return scale;
  }
}PosesVisualData;

class Map : public QObject {
  Q_OBJECT
public:
  explicit Map(const PosesVisualData & worldMapVisualData, QObject * parent = 0);
  ~Map();
  
  void addNewPoses(Drone * drone, const navpts_group::PoseArrayID & posesInfo);
  
private:
  QMap<Drone*, PosesVisualData>_drones;
  
  //QMap<Drone*, navpts::PoseArrayID>_posesByDrone;
  //QMap<Drone*, pcl::PointCloud<pcl::PointXYZ> >_pointClouds;
  QMap<Drone*, QMap<int, geometry_msgs::PoseStamped> >_posesByDrone;
  QMap<int, geometry_msgs::PoseStamped>_worldMap;
  
  QMap<Drone*, cv::Matx44f>_dronesRT;
  
  PosesVisualData _worldMapVisualData;
  
  QMutex _mapMutex;
  
  //for now first drone - map coordinate system's center
  Drone * _droneWorldMap;

  int seq_id;
  
  void addNewDroneRViz(Drone * drone);
  void addNewDroneMap(Drone * drone, navpts_group::PoseArrayID posesInfo);
  void addNewDroneRT(Drone * drone);
  
  static QMap<int, geometry_msgs::PoseStamped> poseArrayToHash(const navpts_group::PoseArrayID & posesInfo);
  static navpts_group::PoseArrayID hashToPoseArray(QMap<int, geometry_msgs::PoseStamped> & posesInfo);
  
  void updateDroneMap(Drone * drone, const navpts_group::PoseArrayID & posesInfo);
  
  void tryToUpdateWorldMap();
  QMap<Drone *, QVector<int> > findMatches(Drone * pivotDrone);
  void findRTMatrices(Drone * pivotDrone, const QMap<Drone *, QVector<int> > &dronesWithMatches);
  
  static cv::Point3f poseToCvPoint(const geometry_msgs::PoseStamped & pose);
  static geometry_msgs::PoseStamped transformAffine(const cv::Matx44f & M, const geometry_msgs::PoseStamped & pose);
  
  void updateRViz();
  
signals:
  void signalUpdateRViz(PosesVisualData posesVisualData, navpts_group::PoseArrayID * posesInfo);
};

#endif
