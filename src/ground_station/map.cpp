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

void Map::addNewPoses(Drone * drone, const navpts_group::PoseArrayID & posesInfo) {
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

void Map::addNewDroneMap(Drone * drone, const navpts_group::PoseArrayID & posesInfo) {
  if (!_posesByDrone.contains(drone)) {
    _posesByDrone.insert(drone, poseArrayToHash(posesInfo));
  }
  else {
    updateDroneMap(drone, posesInfo);
  }
}

QHash<int, geometry_msgs::PoseStamped> Map::poseArrayToHash(const navpts_group::PoseArrayID & posesInfo) {
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

// returns drone list with
QHash<Drone *, QVector<int> > Map::findMatches(Drone * pivotDrone) {
    QHash<Drone*, QVector<int> > dronesWithMatches;
    if (_posesByDrone[pivotDrone].size() >= 4) {

        QHash<int, geometry_msgs::PoseStamped>posesPivotDrone = _posesByDrone[pivotDrone];
        QHashIterator<int, geometry_msgs::PoseStamped>itPivotDrone(posesPivotDrone);
        QHashIterator<Drone*, QHash<int, geometry_msgs::PoseStamped> >itDrone(_posesByDrone);

        while(itDrone.hasNext()) {
            itDrone.next();
            QVector<int> matches;

            // don't want to compute RT matrix for the same drone or to drone with not enough points
            if (pivotDrone != itDrone.key() && itDrone.value().size() >= 4) {
                itPivotDrone.toFront();

                while (itPivotDrone.hasNext())
                {
                    if (itDrone.value().contains(itPivotDrone.key()))
                        matches.push_back(itPivotDrone.key());
                }
                if (matches.size() >= 4) {
                    dronesWithMatches[itDrone.key()] = matches;
                }
            }
        }
    }
    return dronesWithMatches;
}

void Map::findRTMatrices(Drone * pivotDrone, const QHash<Drone *, QVector<int> > & dronesWithMatches) {
    Drone * curDrone;

    QHashIterator<Drone*, QVector<int> >it(dronesWithMatches);
    QHash<int, geometry_msgs::PoseStamped> & pivotDronePoses = _posesByDrone[pivotDrone];
  
    while(it.hasNext()) {

        QHash<int, geometry_msgs::PoseStamped> & curDronePoses = _posesByDrone[it.key()];
        std::vector<cv::Point3f>pivotPts, curPts;
        std::vector<uchar>inliers;
        const QVector<int> & matches = it.value();
        for (int i = 0; i < matches.size(); i++)
        {
            pivotPts.push_back(poseToCvPoint(pivotDronePoses[matches[i]]));
            curPts.push_back(poseToCvPoint(curDronePoses[matches[i]]));
        }

        assert(pivotPts.size() == curPts.size());
        for (int i = 0; i < pivotPts.size(); i++)
            std::cout << pivotPts[i] << "  :  " << curPts[i] << endl;

        cv::Mat curRT34;
        cv::estimateAffine3D(pivotPts, curPts, curRT34, inliers, 5.0, 0.96);

        //cv::Matx44f curRT = cv::Matx44f::eye();
        //_dronesRT.insert(curDrone, curRT);
    }
}

void Map::tryToUpdateWorldMap() {
  QHash<Drone*, QVector<int> >dronesWithMatches = findMatches(_droneWorldMap);
  findRTMatrices(_droneWorldMap, dronesWithMatches);
}

void Map::updateDroneMap(Drone * drone, const navpts_group::PoseArrayID & posesInfo) {
  QHash<int, geometry_msgs::PoseStamped> & dronePoses = _posesByDrone[drone];
  
  navpts_group::PoseID pose;
  
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

cv::Point3f Map::poseToCvPoint(const geometry_msgs::PoseStamped &pose)
{
    return cv::Point3f(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

void Map::updateRViz() {
  QHashIterator<Drone*, PosesVisualData>it(_drones);
  
  while(it.hasNext()) {
    it.next();
    emit signalUpdateRViz(it.value(), hashToPoseArray(_posesByDrone[it.key()]));
  }
  
  emit signalUpdateRViz(_worldMapVisualData, hashToPoseArray(_worldMap));
}
