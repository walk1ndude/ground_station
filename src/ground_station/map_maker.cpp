#include <QtCore/QTime>

#include "ground_station/map_maker.h"

MapMaker::MapMaker(ros::NodeHandle * nh, QObject * parent) :
  _nh(nh),
  QObject(parent) {
}

MapMaker::~MapMaker() {
  
}

void MapMaker::startMapMaker() {
  qsrand(QTime::currentTime().msec());
  fetchMap();
  fetchPublishers();
}

void MapMaker::fetchMap() {
  _map = new Map(PosesVisualData("WorldMap"));
  QObject::connect(_map, &Map::signalUpdateRViz, this, &MapMaker::updateRVizMap);
}

void MapMaker::fetchPublishers() {
  _visualPub = _nh->advertise<visualization_msgs::MarkerArray>(MAP_TOPIC, 1);
}

void MapMaker::updateRVizMap(PosesVisualData posesVisualData, geometry_msgs::PoseArray posesInfo) {
  visualization_msgs::MarkerArray mapPoses;
  
  mapPoses.markers.resize(posesInfo.poses.size());
  for (size_t i = 0; i != posesInfo.poses.size(); ++ i) {
    mapPoses.markers[i].ns = posesVisualData.ns;
    mapPoses.markers[i].type = posesVisualData.type;
    mapPoses.markers[i].lifetime = posesVisualData.lifetime;
    mapPoses.markers[i].action = posesVisualData.action;
    mapPoses.markers[i].color = posesVisualData.color;
    mapPoses.markers[i].scale = posesVisualData.scale;
    
    mapPoses.markers[i].id = i;
    mapPoses.markers[i].header.frame_id = "map_frame";
    mapPoses.markers[i].header.stamp = ros::Time();
    mapPoses.markers[i].pose = posesInfo.poses[i];
  }
  
  _visualPub.publish<visualization_msgs::MarkerArray>(mapPoses);
}

void MapMaker::addNewInfoToMap(Map * map, Drone * drone, const navpts_group::PoseArrayID & posesInfo) {
  map->addNewPoses(drone, posesInfo);
}

void MapMaker::correctPosesInfo(Drone * drone, navpts_group::PoseArrayID posesInfo) {
  addNewInfoToMap(_map, drone, posesInfo);
}
