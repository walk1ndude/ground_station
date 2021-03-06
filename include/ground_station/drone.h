#ifndef DRONEH
#define DRONEH

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <navpts_group/PoseArrayID.h>

#include <QtCore/QObject>
#include <QtCore/QProcess>
#include <QtCore/QDebug>

#define GET_MARKER_INFO_TOPIC "/get_poses_info"
#define SET_MARKER_INFO_TOPIC "/set_poses_info"

typedef struct _DroneData {
  int id;
  std::string string_id; //for rviz
  std::string driver;
  
  _DroneData(int id, std::string string_id, std::string driver) :
    id(id),
    string_id(string_id),
    driver(driver) {};
}DroneData;

class Drone : public QObject {
  Q_OBJECT
public:
  Drone(const DroneData & droneData, ros::NodeHandle * nh, QObject * parent = 0);
  ~Drone();
  
  int id();
  std::string strId();
  
private:
  ros::NodeHandle * _nh;
  
  ros::Subscriber _posesInfoSubscriber;
  
  ros::Publisher _posesInfoPublisher;
  
  navpts_group::PoseArrayID _posesInfo;
  
  DroneData _droneData;
  
  QProcess * _process;
  
  QString _program;
  
  void fetchDrone();
  void fetchProgram();
  void fetchSubscribers();
  void fetchPublishers();
  
  void getPosesInfo(const navpts_group::PoseArrayID & posesInfo);

signals:
  void signalTaskFinished(Drone * drone);
  void signalCorrectPosesInfo(Drone * drone, navpts_group::PoseArrayID posesInfo);

public slots:
  void startTask();
  void finishTask(int code);
  void setPosesInfo(navpts_group::PoseArrayID posesInfo);
};

#endif
