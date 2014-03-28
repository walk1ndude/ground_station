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
    _markersPub = _nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void MapMaker::updateRVizMap(PosesVisualData posesVisualData, navpts_group::PoseArrayID * posesInfo) {
    //ROS_INFO("updateRVizMap");

    nav_msgs::Path path;
    const std::string path_topic = std::string("/path_") + posesVisualData.ns;
    path.header.frame_id = path_topic;

    for (size_t i = 0; i != posesInfo->poses.size(); ++ i) {

        if (posesInfo->poses[i].id < DRONEPOSE_SEQ_ID)
        {
            visualization_msgs::Marker pose;

            pose.ns = posesVisualData.ns;
            pose.type = posesVisualData.type;
            pose.lifetime = posesVisualData.lifetime;
            pose.action = posesVisualData.action;
            pose.color = posesVisualData.color;
            pose.scale = posesVisualData.scale;

            pose.id = posesInfo->poses[i].id;
            pose.header.frame_id = "/map_frame";
            pose.header.stamp = ros::Time();
            pose.pose = posesInfo->poses[i].spottedPose.pose;
            _markersPub.publish<visualization_msgs::Marker>(pose);
        }
        else
            path.poses.push_back(posesInfo->poses[i].spottedPose);
    }
    if (path.poses.size())
    {
        if (!_pathPubs.contains(path_topic))
            _pathPubs.insert(path_topic, _nh->advertise<nav_msgs::Path>(path_topic, 1));
        _pathPubs[path_topic].publish<nav_msgs::Path>(path);
    }
    delete posesInfo;
    //ROS_INFO("updateRVizMap_end");
}

void MapMaker::addNewInfoToMap(Map * map, Drone * drone, const navpts_group::PoseArrayID & posesInfo) {
  map->addNewPoses(drone, posesInfo);
}

void MapMaker::correctPosesInfo(Drone * drone, navpts_group::PoseArrayID posesInfo) {
  addNewInfoToMap(_map, drone, posesInfo);
}
