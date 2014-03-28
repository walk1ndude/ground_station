#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ground_station/map.h"

Map::Map(const PosesVisualData & worldMapVisualData, QObject * parent):
  _worldMapVisualData(worldMapVisualData),
  _droneWorldMap(0),
  seq_id(0),
  QObject(parent) {

}

Map::~Map() {
  _drones.clear();
  _posesByDrone.clear();
  _dronesRT.clear();
}

void Map::addNewPoses(Drone * drone, const navpts_group::PoseArrayID & posesInfo) {
    //ROS_INFO("addNewPoses");
    _mapMutex.lock();

    addNewDroneRViz(drone);
    addNewDroneRT(drone);
    addNewDroneMap(drone, posesInfo);

    tryToUpdateWorldMap();

    updateRViz();

    _mapMutex.unlock();
    //ROS_INFO("addNewPoses_end");
}

void Map::addNewDroneRViz(Drone * drone) { 
  if (!_drones.contains(drone)) {
    visualization_msgs::Marker::_color_type color;
    
    if (_drones.empty()) {
      color.a = 1.0;
      color.r = 1.0;
      color.b = 0.0;
      color.g = 0.0;
    }
    else {
      color.a = 1.0;
      color.r = 0.0;
      color.b = 0.0;
      color.g = 1.0;
    }
    
    _drones.insert(drone, PosesVisualData(drone->strId(), color));
  }
}

void Map::addNewDroneRT(Drone * drone) {
  if (!_dronesRT.contains(drone)) {
    if (!_droneWorldMap) {
      _droneWorldMap = drone;
    }
    cv::Matx44f RT = cv::Matx44f::eye();
    RT(1,3) = _dronesRT.size() * 10;
    _dronesRT.insert(drone, RT);
  }
}

void Map::addNewDroneMap(Drone * drone, navpts_group::PoseArrayID posesInfo) {
    posesInfo.poses[0].id = DRONEPOSE_SEQ_ID + seq_id++;
    if (!_posesByDrone.contains(drone)) {
        _posesByDrone.insert(drone, poseArrayToHash(posesInfo));
    }
    else {
        updateDroneMap(drone, posesInfo);
    }
}

QMap<int, geometry_msgs::PoseStamped> Map::poseArrayToHash(const navpts_group::PoseArrayID & posesInfo) {
  QMap<int, geometry_msgs::PoseStamped>poses;
  
  for (size_t i = 0; i != posesInfo.poses.size(); ++ i) {
    poses.insert(posesInfo.poses[i].id, posesInfo.poses[i].spottedPose);
  }
  
  return poses;
}

navpts_group::PoseArrayID Map::hashToPoseArray(QMap<int, geometry_msgs::PoseStamped> & posesInfo) {
    //ROS_INFO("hashToPoseArray");

    navpts_group::PoseArrayID poseArray;

    QMapIterator<int, geometry_msgs::PoseStamped>it(posesInfo);
    while (it.hasNext()) {
        it.next();
        navpts_group::PoseID pose;
        pose.id = it.key();
        pose.spottedPose.pose = it.value().pose;
        poseArray.poses.push_back(pose);
    }

    //ROS_INFO("hashToPoseArray_end");
    return poseArray;
}

// returns drone list with
QMap<Drone *, QVector<int> > Map::findMatches(Drone * pivotDrone) {
    //ROS_INFO("findMatches");
    QMap<Drone*, QVector<int> > dronesWithMatches;

    QMap<int, geometry_msgs::PoseStamped>posesPivotDrone = _posesByDrone[pivotDrone];
    QMapIterator<int, geometry_msgs::PoseStamped>itPivotDrone(posesPivotDrone);
    QMapIterator<Drone*, QMap<int, geometry_msgs::PoseStamped> >itDrone(_posesByDrone);

    while(itDrone.hasNext()) {
        itDrone.next();
        QVector<int> matches;

        // don't want to compute RT matrix for the same drone or to drone with not enough points
        if (pivotDrone != itDrone.key()) { // 4 + drone
            itPivotDrone.toFront();

            while (itPivotDrone.hasNext())
            {
                itPivotDrone.next();
                if (itPivotDrone.key() >= DRONEPOSE_SEQ_ID) break;
                if (itDrone.value().contains(itPivotDrone.key()))
                    matches.push_back(itPivotDrone.key());
            }
            if (matches.size() >= 4) {
		qDebug() << "4 points matched";
                dronesWithMatches.insert(itDrone.key(), matches);
            }
        }
    }

    //ROS_INFO("findMatches_end");
    return dronesWithMatches;
}

void Map::findRTMatrices(Drone * pivotDrone, const QMap<Drone *, QVector<int> > & dronesWithMatches) {
    //ROS_INFO("findRTMatrices");
    Drone * curDrone;

    QMapIterator<Drone*, QVector<int> >it(dronesWithMatches);
    QMap<int, geometry_msgs::PoseStamped> & pivotDronePoses = _posesByDrone[pivotDrone];
  
    while(it.hasNext()) {
        it.next();

        //ROS_INFO("EEEEEEEEEEEEEEEEE!!!");
        // dronesWithMatches already contains matches with enough size

        QMap<int, geometry_msgs::PoseStamped> & curDronePoses = _posesByDrone[it.key()];
        //ROS_INFO("1...");
        std::vector<cv::Point3f>pivotPts, curPts;
        std::vector<uchar>inliers;
        const QVector<int> & matches = it.value();
        for (int i = 0; i < matches.size(); i++)
        {
            pivotPts.push_back(poseToCvPoint(pivotDronePoses[matches[i]]));
            curPts.push_back(poseToCvPoint(curDronePoses[matches[i]]));
        }

        //ROS_INFO("123");
        assert(pivotPts.size() == curPts.size());
        //ROS_INFO("222");

        cv::Mat1d RT;
        cv::estimateAffine3D(pivotPts, curPts, RT, inliers, 5.0, 0.96);
        //ROS_INFO("321");

        //cv::Matx44f curRT = cv::Matx44f::eye();
        _dronesRT[it.key()] = cv::Matx44f(RT(0,0), RT(0,1), RT(0,2), RT(0,3),
                                          RT(1,0), RT(1,1), RT(1,2), RT(1,3),
                                          RT(2,0), RT(2,1), RT(2,2), RT(2,3),
                                          0,       0,       0,       1).inv();
        //ROS_INFO("111");
        //_dronesRT.insert(curDrone, curRT);
    }
    //ROS_INFO("findRTMatrices_end");
}

void Map::tryToUpdateWorldMap() {
    QMap<Drone*, QVector<int> >dronesWithMatches = findMatches(_droneWorldMap);
    findRTMatrices(_droneWorldMap, dronesWithMatches);
}

void Map::updateDroneMap(Drone * drone, const navpts_group::PoseArrayID & posesInfo) {
  //ROS_INFO("updateDroneMap");
  QMap<int, geometry_msgs::PoseStamped> & dronePoses = _posesByDrone[drone];
  
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
  //ROS_INFO("updateDroneMap_end");
}

cv::Point3f Map::poseToCvPoint(const geometry_msgs::PoseStamped &pose)
{
    return cv::Point3f(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

geometry_msgs::PoseStamped Map::transformAffine(const cv::Matx44f & M, const geometry_msgs::PoseStamped & pose)
{
    cv::Matx41f newPos = M * cv::Matx41f(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1);

    geometry_msgs::PoseStamped p;
    p.header = pose.header;

    p.pose.position.x = newPos(0,0)/newPos(3,0);
    p.pose.position.y = newPos(1,0)/newPos(3,0);
    p.pose.position.z = newPos(2,0)/newPos(3,0);

    return p;
}

void Map::updateRViz() {
  //ROS_INFO("updateRViz");
  QMapIterator<Drone*, PosesVisualData>it(_drones);
  
  while(it.hasNext()) {
    it.next();
    navpts_group::PoseArrayID * poses = new navpts_group::PoseArrayID;
    *poses = hashToPoseArray(_posesByDrone[it.key()]);

    // perform coordinates translation
    cv::Matx44f & RT = _dronesRT[it.key()];
    for (int i = 0; i < poses->poses.size(); i++)
        poses->poses[i].spottedPose = transformAffine(RT, poses->poses[i].spottedPose);

    emit signalUpdateRViz(it.value(), poses);
  }
  
  //emit signalUpdateRViz(_worldMapVisualData, hashToPoseArray(_worldMap));
  //ROS_INFO("updateRViz_end");
}
