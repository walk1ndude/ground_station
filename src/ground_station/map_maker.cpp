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
/*
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
   // delete posesInfo;
    //ROS_INFO("updateRVizMap_end");*/
  visualization_msgs::Marker pts;
    pts.type = visualization_msgs::Marker::POINTS;
    pts.header.frame_id = "/map_frame";
    pts.header.stamp = ros::Time();
    pts.id = 0;
    pts.ns = posesVisualData.ns;
    pts.color = posesVisualData.color;
    pts.scale = posesVisualData.scale;
    pts.action = posesVisualData.action;
    pts.lifetime = posesVisualData.lifetime;
    pts.pose.position.x = pts.pose.position.y = pts.pose.position.z = 0;
    pts.pose.orientation.x = pts.pose.orientation.y = pts.pose.orientation.z = 0;

    visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.header.frame_id = "/map_frame";
    path.header.stamp = ros::Time();
    path.id = 1;
    path.ns = posesVisualData.ns;
    path.color = posesVisualData.color;
    path.scale = posesVisualData.scale;
    path.action = posesVisualData.action;
    path.lifetime = posesVisualData.lifetime;

    for (size_t i = 0; i != posesInfo->poses.size(); ++ i) {

        geometry_msgs::Point p;
        p.x = posesInfo->poses[i].spottedPose.pose.position.x;
        p.y = posesInfo->poses[i].spottedPose.pose.position.y;
        p.z = posesInfo->poses[i].spottedPose.pose.position.z;

        if (posesInfo->poses[i].id < DRONEPOSE_SEQ_ID)
            pts.points.push_back(p);
        else
            path.points.push_back(p);
    }

    if (pts.points.size() > 0)
        _markersPub.publish<visualization_msgs::Marker>(pts);
    
    if (path.points.size() > 0)
        _markersPub.publish<visualization_msgs::Marker>(path);
    
    delete posesInfo;
    //ROS_INFO("updateRVizMap_end");
}

void MapMaker::addNewInfoToMap(Map * map, Drone * drone, const navpts_group::PoseArrayID & posesInfo) {
  map->addNewPoses(drone, posesInfo);
}

void MapMaker::correctPosesInfo(Drone * drone, navpts_group::PoseArrayID posesInfo) {
  addNewInfoToMap(_map, drone, posesInfo);
}
