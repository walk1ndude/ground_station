#ifndef DRONEH
#define DRONEH

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <QtCore/QObject>
#include <QtCore/QProcess>
#include <QtCore/QDebug>

#define GET_MARKER_INFO_TOPIC "/get_marker_info"
#define SET_MARKER_INFO_TOPIC "/set_marker_info"

typedef struct _DroneData {
  int id;
  std::string driver;
}DroneData;

class Drone : public QObject {
  Q_OBJECT
public:
  Drone(const DroneData & droneData, ros::NodeHandle * nh, QObject * parent = 0);
  ~Drone();
  
private:
  ros::NodeHandle * _nh;
  
  ros::Subscriber _markerInfoSubscriber;
  
  ros::Publisher _markerInfoPublisher;
  
  geometry_msgs::PoseArray _markerInfo;
  
  DroneData _droneData;
  
  QProcess * _process;
  
  QString _program;
  
  void fetchProgram();
  void fetchSubscribers();
  void fetchPublishers();
  
  void getMarkerInfo(const geometry_msgs::PoseArray & markerInfo);

  
signals:
  void signalTaskFinished(Drone * drone);
  void signalCorrectMarkerInfo(geometry_msgs::PoseArray markerInfo);

public slots:
  void startTask();
  void finishTask(int code);
  void correctMarkerInfo(geometry_msgs::PoseArray markerInfo);
};

#endif