#ifndef GROUND_STATIONH
#define GROUND_STATIONH

#include <QtCore/QThread>
#include <QtCore/QSet>

#include "ground_station/drone.h"
#include "ground_station/map_maker.h"

class GroundStation : public QObject {
  Q_OBJECT
public:
  explicit GroundStation(QObject * parent = 0);
  ~GroundStation();
  
  void loop(const int & frequency);
  
private:
  ros::NodeHandle _nh;
  ros::NodeHandle _nhPrivate;
  
  ros::Subscriber imageSubscriber;
  
  MapMaker * _mapMaker;
  
  QSet<Drone*>_drones;
  
  void fetchParams();
  void fetchMapMaker();
  
  void fetchDrones(const std::vector<std::string> & droneDrivers);

public slots:
  void launchDrones();
  void removeDrone(Drone * drone);
};

#endif