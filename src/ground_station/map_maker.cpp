#include <QtCore/QTime>

#include "ground_station/map_maker.h"

MapMaker::MapMaker(ros::NodeHandle * nh, QObject * parent) :
  _nh(nh),
  QObject(parent) {
}

MapMaker::~MapMaker() {
  _drones.clear();
}

void MapMaker::startMapMaker() {
  qsrand(QTime::currentTime().msec());
  fetchPublishers();
}

void MapMaker::fetchPublishers() {
  _visualPub = _nh->advertise<visualization_msgs::MarkerArray>(MAP_TOPIC, 1);
}

void MapMaker::updateRVizMap(const std::string ns, const visualization_msgs::Marker::_color_type & color,
			     const geometry_msgs::PoseArray & markerInfo) {
  
  visualization_msgs::MarkerArray mapPoints;
  
  mapPoints.markers.resize(markerInfo.poses.size());
  for (size_t i = 0; i != markerInfo.poses.size(); ++ i) {
    mapPoints.markers[i].ns = ns;
    mapPoints.markers[i].id = i;
    mapPoints.markers[i].header.frame_id = "map_frame";
    mapPoints.markers[i].type = visualization_msgs::Marker::SPHERE;
    mapPoints.markers[i].lifetime = ros::Duration();
    mapPoints.markers[i].header.stamp = ros::Time();
    mapPoints.markers[i].action = visualization_msgs::Marker::ADD;
    mapPoints.markers[i].color = color;
    mapPoints.markers[i].pose = markerInfo.poses[i];
    mapPoints.markers[i].scale.x = 0.1;
    mapPoints.markers[i].scale.y = 0.1;
    mapPoints.markers[i].scale.z = 0.1;
  }
  
  _visualPub.publish<visualization_msgs::MarkerArray>(mapPoints);
}

void MapMaker::addNewDrone(Drone * drone) { 
  if (!_drones.contains(drone)) {
    visualization_msgs::Marker::_color_type color;
    
    color.a = 1.0;
    color.b = qrand() / (float) RAND_MAX;
    color.g = qrand() / (float) RAND_MAX;
    color.r = qrand() / (float) RAND_MAX;
    
    _drones.insert(drone, color);
  }
}

void MapMaker::correctMarkerInfo(Drone * drone, geometry_msgs::PoseArray markerInfo) {
  addNewDrone(drone);
  updateRVizMap(drone->strId(), _drones[drone], markerInfo);
}
