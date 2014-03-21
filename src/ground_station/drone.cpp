#include <QtCore/QDir>
#include <QtCore/QTextStream>

#include "ground_station/drone.h"

Drone::Drone(const DroneData & droneData, ros::NodeHandle * nh, QObject * parent) :
  QObject(parent),
  _nh(nh),
  _droneData(droneData) {
  fetchProgram();
}

Drone::~Drone() {
  delete _process;
}

void Drone::fetchSubscribers() {
  _markerInfoSubscriber = _nh->subscribe("/drone" + std::to_string(_droneData.id) + GET_MARKER_INFO_TOPIC, 1, &Drone::getMarkerInfo, this);
}

void Drone::fetchPublishers() {
  _markerInfoPublisher = _nh->advertise<geometry_msgs::PoseArray>("/drone" + std::to_string(_droneData.id) + SET_MARKER_INFO_TOPIC, 1);
}

void Drone::fetchProgram() {
  QString droneName = "drone" + QString::number(_droneData.id);
  
  QString fileName = QString("%1/.ros/%2.launch").arg(QDir::homePath()).arg(droneName);
  QFile file(fileName);
  
  _program = "roslaunch " + fileName;
  
  if (file.open(QIODevice::WriteOnly)) {
    QTextStream stream(&file);
  
    stream << QString("<launch>\n<group ns=\"%1\">\n").arg(droneName) <<
    QString("<remap from=\"/ardrone/image_raw\" to=\"/%1/ardrone/image_raw\" />\n").arg(droneName) <<
    QString("<remap from=\"/ardrone/navdata\" to=\"/%1/ardrone/navdata\" />\n").arg(droneName) <<
    QString("<remap from=\"/get_marker_info\" to=\"/%1/get_marker_info\" />\n").arg(droneName) <<
    QString("<remap from=\"cmd_vel\" to=\"/%1/cmd_vel\" />\n").arg(droneName) <<
    QString("<%2 output=\"screen\"/>\n").arg(QString::fromStdString(_droneData.driver)) <<
    "<node pkg=\"navpts\" type=\"navpts\" name=\"navpts\" respawn=\"true\" output=\"screen\">\n" <<
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
  if (_program != "") {
    _process = new QProcess;
    _process->setProcessChannelMode(QProcess::MergedChannels);
    _process->start(_program);

    _process->waitForStarted();

    _process->waitForFinished(-1);

    emit signalTaskFinished(this);
  }
}

void Drone::getMarkerInfo(const geometry_msgs::PoseArray & markerInfo) {
  _markerInfo = markerInfo;
  
  emit signalMarkerInfo(markerInfo);
}

void Drone::correctMarkerInfo(geometry_msgs::PoseArray markerInfo) {
  
  
}
