#include "ground_station/map.h"

Map::Map(const PosesVisualData & posesVisualData, QObject * parent):
  _posesVisualData(posesVisualData),
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

void Map::updateDroneMap(Drone * drone, const geometry_msgs::PoseArray & posesInfo) {

}

void Map::updateRViz() {
  QHashIterator<Drone*, PosesVisualData>it(_drones);
  
  while(it.hasNext()) {
    it.next();
    emit signalUpdateRViz(it.value(), qVPosesArrayToPoseArray(_posesByDrone[it.key()]));
  }
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
