#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ground_station/map.h"

Map::Map(const PosesVisualData & worldMapVisualData, QObject * parent):
  _worldMapVisualData(worldMapVisualData),
  _droneWorldMap(0),
  QObject(parent) {

}

Map::~Map() {
  _drones.clear();
  _posesByDrone.clear();
  _dronesRT.clear();
}

void Map::addNewPoses(Drone * drone, const navpts::PoseArrayID & posesInfo) {
  _mapMutex.lock();
  
  addNewDroneRViz(drone);
  addNewDroneRT(drone);
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

void Map::addNewDroneRT(Drone * drone) {
  if (!_dronesRT.contains(drone)) {
    if (!_droneWorldMap) {
      _droneWorldMap = drone;
    }
    _dronesRT.insert(drone, cv::Matx44f::eye());
  }
}

void Map::addNewDroneMap(Drone * drone, const navpts::PoseArrayID & posesInfo) {
  if (!_posesByDrone.contains(drone)) {
    _posesByDrone.insert(drone, poseArrayToHash(posesInfo));
  }
  else {
    updateDroneMap(drone, posesInfo);
  }
}

QHash<int, geometry_msgs::PoseStamped> Map::poseArrayToHash(const navpts::PoseArrayID & posesInfo) {
  QHash<int, geometry_msgs::PoseStamped>poses;
  
  for (size_t i = 0; i != posesInfo.poses.size(); ++ i) {
    poses.insert(posesInfo.poses[i].id, posesInfo.poses[i].spottedPose);
  }
  
  return poses;
}

geometry_msgs::PoseArray Map::hashToPoseArray(const QHash<int, geometry_msgs::PoseStamped> & posesInfo) {
  geometry_msgs::PoseArray poseArray;
  
  QHashIterator<int, geometry_msgs::PoseStamped>it(posesInfo);
  
  while (it.hasNext()) {
    it.next();
    poseArray.poses.push_back(it.value().pose);
  }
  
  return poseArray;
}

QVector<Drone*> Map::findDronesWithSimilarPoses(Drone * pivotDrone) {
  if (_posesByDrone[pivotDrone].size() >= 4) {
    QVector<Drone*>dronesWithSimilarPoses;
    
    QHash<int, geometry_msgs::PoseStamped>posesPivotDrone = _posesByDrone[pivotDrone];
    
    QHashIterator<int, geometry_msgs::PoseStamped>itPivotDrone(posesPivotDrone);
    QHashIterator<Drone*, QHash<int, geometry_msgs::PoseStamped> >itDrone(_posesByDrone);
    
    int similarity;
    
    while(itDrone.hasNext()) {
      itDrone.next();
      
      // don't want to compute RT matrix for the same drone or to drone with not enough points
      if (pivotDrone != itDrone.key() && itDrone.value().size() >= 4) {
	itPivotDrone.toFront();
	similarity = 0;

	if (posesPivotDrone.size() >= itDrone.value().size()) {
	  while(similarity < 4 && itPivotDrone.hasNext()) {
	    itPivotDrone.next();
	    similarity += (itDrone.value().contains(itPivotDrone.key())) ? 1 : 0;
	  }
	}
	else {
	  QHashIterator<int, geometry_msgs::PoseStamped>itCurDrone(itDrone.value());
	  while(similarity < 4 && itCurDrone.hasNext()) {
	    itCurDrone.next();
	    similarity += (posesPivotDrone.contains(itCurDrone.key())) ? 1 : 0;
	  }
	}
	
	qDebug() << "push new drone similar";
	if (similarity >= 4) {
	  dronesWithSimilarPoses.push_back(itDrone.key());
	}
      }
    }
    
    return dronesWithSimilarPoses;
  }
  else {
    qDebug() << "not enough points";
    return QVector<Drone*>(); //not enough points -> try on next update
  }
}

void Map::findRTMatrices(Drone * pivotDrone, const QVector<Drone*> & dronesWithSimilarPoses) {
  Drone * curDrone;
  
  QVectorIterator<Drone*>it(dronesWithSimilarPoses);
  
  std::vector<cv::Point3f>pivotPoses = Map::posesToCvPoints(_posesByDrone[pivotDrone]);
  
  std::vector<cv::Point3f>curPoses;
  std::vector<uchar>inliers;
  
  cv::Matx44f curRT = cv::Matx44f::eye();
  cv::Mat curRT34(3, 4, CV_64F);
  
  while(it.hasNext()) {
    curDrone = it.next();
    curPoses = Map::posesToCvPoints(_posesByDrone[curDrone]);
    
    cv::estimateAffine3D(pivotPoses, curPoses, curRT34, inliers, 5.0, 0.96);
    //_dronesRT.insert(curDrone, curRT);
  }
}

void Map::tryToUpdateWorldMap() {
  QVector<Drone*>dronesWithSimilarPoses = findDronesWithSimilarPoses(_droneWorldMap);
  qDebug() << dronesWithSimilarPoses;
  
  if (dronesWithSimilarPoses.size()) {
    qDebug() << "similarity found";
    findRTMatrices(_droneWorldMap, dronesWithSimilarPoses);
  }
}

void Map::updateDroneMap(Drone * drone, const navpts::PoseArrayID & posesInfo) {
  QHash<int, geometry_msgs::PoseStamped> & dronePoses = _posesByDrone[drone];
  
  navpts::PoseID pose;
  
  for (size_t i = 0; i != posesInfo.poses.size(); ++ i) {
    pose = posesInfo.poses[i];
    
    if (dronePoses.contains(pose.id)) {
      dronePoses[pose.id] = pose.spottedPose;
    }
    else {
      dronePoses.insert(pose.id, pose.spottedPose);
    }
  }
}

std::vector<cv::Point3f> Map::posesToCvPoints(const QHash<int, geometry_msgs::PoseStamped> & poses) {
  std::vector<cv::Point3f>cvPoints;
  
  QHashIterator<int, geometry_msgs::PoseStamped>it(poses);
  
  while(it.hasNext()) {
    it.next();
    cvPoints.push_back(cv::Point3f(it.value().pose.position.x, it.value().pose.position.y, it.value().pose.position.z));
  }
  
  return cvPoints;
}

void Map::updateRViz() {
  QHashIterator<Drone*, PosesVisualData>it(_drones);
  
  while(it.hasNext()) {
    it.next();
    emit signalUpdateRViz(it.value(), hashToPoseArray(_posesByDrone[it.key()]));
  }
  
  emit signalUpdateRViz(_worldMapVisualData, hashToPoseArray(_worldMap));
}