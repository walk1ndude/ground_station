#include <QtCore/QDir>
#include <QtCore/QTextStream>

#include <sstream>

#include "ground_station/drone.h"

Drone::Drone(const DroneData & droneData, ros::NodeHandle * nh, QObject * parent) :
  QObject(parent),
  _nh(nh),
  _droneData(droneData) {
    
}

Drone::~Drone() {
  delete _process;
}

void Drone::fetchDrone() {
  fetchPublishers();
  fetchSubscribers();
  fetchProgram();
}

int Drone::id() {
  return _droneData.id;
}

std::string Drone::strId() {
  return _droneData.string_id;
}

void Drone::fetchSubscribers() {
  std::stringstream topicToSubsribe;
  
  topicToSubsribe << "/drone" << _droneData.id << GET_MARKER_INFO_TOPIC;
  
  _posesInfoSubscriber = _nh->subscribe(topicToSubsribe.str(), 1, &Drone::getPosesInfo, this);
}

void Drone::fetchPublishers() {
  std::stringstream topicToPublish;
  
  topicToPublish << "/drone" << _droneData.id << SET_MARKER_INFO_TOPIC;
  
  _posesInfoPublisher = _nh->advertise<navpts_group::PoseArrayID>(topicToPublish.str(), 1);
}

void Drone::fetchProgram() {
  QString droneName = "drone" + QString::number(_droneData.id);
  
  QString fileName = QString("%1/.ros/%2.launch").arg(QDir::homePath()).arg(droneName);
  QFile file(fileName);
  
  _program = "roslaunch " + fileName;
  
  if (file.open(QIODevice::WriteOnly)) {
    QTextStream stream(&file);
  
    stream << QString("<launch>\n<group ns=\"%1\">\n").arg(droneName) <<
    QString("<remap from=\"/ardrone/front/image_raw\" to=\"/%1/ardrone/front/image_raw\" />\n").arg(droneName) <<
    QString("<remap from=\"/ardrone/bottom/image_raw\" to=\"/%1/ardrone/bottom/image_raw\" />\n").arg(droneName) <<
    QString("<remap from=\"/ardrone/image_raw\" to=\"/%1/ardrone/image_raw\" />\n").arg(droneName) <<
    QString("<remap from=\"/ardrone/navdata\" to=\"/%1/ardrone/navdata\" />\n").arg(droneName) <<
    QString("<remap from=\"/get_poses_info\" to=\"/%1/get_poses_info\" />\n").arg(droneName) <<
    QString("<remap from=\"cmd_vel\" to=\"/%1/cmd_vel\" />\n").arg(droneName) <<
    QString("<%2 output=\"screen\"/>\n").arg(QString::fromStdString(_droneData.driver)) <<
    "<node pkg=\"navpts_group\" type=\"navpts_group\" name=\"navpts_group\" respawn=\"true\" output=\"screen\">\n" <<
    "<rosparam param=\"flightTask\">[1, 4, 5]</rosparam>\n" <<
    "<rosparam param=\"yawRotateHeight\">0.45</rosparam>\n" <<
    "<rosparam param=\"hitTargetDist\">0.15</rosparam>\n" <<
    "<rosparam param=\"nearToTargetDist\">1.0</rosparam>\n" <<
    "<rosparam param=\"targetHeightHeading\">1.35</rosparam>\n" <<
    "<rosparam param=\"timeBetweenAttempts\">0.8</rosparam>\n" <<
    "</node>\n</group>\n</launch>";
    
    file.close();
  }
}

void Drone::startTask() {
  fetchDrone();
  
  if (_program != "") {
    _process = new QProcess;
    _process->setProcessChannelMode(QProcess::MergedChannels);

    _process->start(_program);
    
    QObject::connect(_process, (void (QProcess::*)(int))&QProcess::finished, this, &Drone::finishTask);

    _process->waitForStarted();
  }
}

void Drone::finishTask(int code) {
  emit signalTaskFinished(this);
}

void Drone::getPosesInfo(const navpts_group::PoseArrayID & posesInfo) {
  _posesInfo = posesInfo;
  emit signalCorrectPosesInfo(this, posesInfo);
}

void Drone::setPosesInfo(navpts_group::PoseArrayID posesInfo) {
  _posesInfoPublisher.publish<navpts_group::PoseArrayID>(posesInfo);
}
