#include "ground_station/map.h"

Map::Map(const PosesVisualData & worldMapVisualData, QObject * parent):
  _worldMapVisualData(worldMapVisualData),
  QObject(parent) {

}

Map::~Map() {
  _drones.clear();
  _posesByDrone.clear();
}

void Map::addNewPoses(Drone * drone, const geometry_msgs::PoseArray & posesInfo) {
  _mapMutex.lock();
  
  addNewDroneRViz(drone);
  addNewDroneMap(drone, posesInfo);
  
  tryToUpdateWorldMap();
  
  updateRViz();
  
  _mapMutex.unlock();
}

void Map::addNewDroneRViz(Drone * drone) { 
  if (!_drones.contains(drone)) {
    _drones.insert(drone, PosesVisualData(drone->strId()));
  }
}

void Map::addNewDroneMap(Drone * drone, const geometry_msgs::PoseArray & posesInfo) {
  if (!_posesByDrone.contains(drone)) {
    _posesByDrone.insert(drone, posesArrayToQVPoseArray(posesInfo));
  }
  else {
    updateDroneMap(drone, posesInfo);
  }
}

void Map::tryToUpdateWorldMap() {
  
  
}

void Map::updateDroneMap(Drone * drone, const geometry_msgs::PoseArray & posesInfo) {
  // 'cause we don't want to check new poses with new, only new with old ones
  QVector<geometry_msgs::PoseStamped>posesDroneNew = _posesByDrone[drone];
  QVector<geometry_msgs::PoseStamped>posesDroneOld = _posesByDrone[drone];
  
  geometry_msgs::PoseStamped pose;
  size_t pos;
  
  for (size_t i = 0; i != posesInfo.poses.size(); ++ i) {
    pos = findPoseInArray(posesDroneOld, posesInfo.poses[i], POSE_SIMILAR_TOLERANCE_BORDER);
    
    if (pos == -1) {
      pose.header = posesInfo.header;
      pose.pose = posesInfo.poses[i];
      
      posesDroneNew.push_back(pose);
    }
    else {
      posesDroneNew[pos].header = posesInfo.header;
      posesDroneNew[pos].pose = posesInfo.poses[i];
    }
  }
  
  _posesByDrone[drone] = posesDroneNew;
}

size_t Map::findPoseInArray(const QVector<geometry_msgs::PoseStamped> & posesDrone, const geometry_msgs::Pose & pose,
		       const float & toleranceRadius) {
  geometry_msgs::PoseStamped curPose;
  
  for (size_t i = 0; i != posesDrone.size(); ++ i) {
    if ((curPose.pose.position.x - pose.position.x) * (curPose.pose.position.x - pose.position.x) +
      (curPose.pose.position.y - pose.position.y) * (curPose.pose.position.y - pose.position.y) +
      (curPose.pose.position.z - pose.position.z) * (curPose.pose.position.z - pose.position.z) <= toleranceRadius) {
	return i;
    }
  }
  
  return -1;
}

void Map::updateRViz() {
  QHashIterator<Drone*, PosesVisualData>it(_drones);
  
  while(it.hasNext()) {
    it.next();
    emit signalUpdateRViz(it.value(), qVPosesArrayToPoseArray(_posesByDrone[it.key()]));
  }
  
  emit signalUpdateRViz(_worldMapVisualData, qVPosesArrayToPoseArray(_worldMap));
}

QVector<geometry_msgs::PoseStamped> Map::posesArrayToQVPoseArray(const geometry_msgs::PoseArray & posesArray) {
  QVector<geometry_msgs::PoseStamped>qVPosesArray;
  geometry_msgs::PoseStamped pStamped;
  
  for (size_t i = 0; i != posesArray.poses.size(); ++ i) {
    pStamped.header = posesArray.header;
    pStamped.pose = posesArray.poses[i];
    
    qVPosesArray.push_back(pStamped);
  }
  
  return qVPosesArray;
}

geometry_msgs::PoseArray Map::qVPosesArrayToPoseArray(const QVector<geometry_msgs::PoseStamped> & posesStamped) {
  geometry_msgs::PoseArray poseArray;
  poseArray.header = posesStamped[0].header;
    
  for (size_t i = 0; i != posesStamped.size(); ++ i) {
    poseArray.poses.push_back(posesStamped[i].pose);
  }
  
  return poseArray;
}
