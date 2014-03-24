#include <QtCore/QCoreApplication>

#include "ground_station/ground_station.h"

int main(int argc, char ** argv) {
  ros::init(argc, argv, "ground_station_node");
  
  QCoreApplication app(argc,argv);
  
  GroundStation groundStation;
  
  groundStation.loop(30);
  
  return app.exec();
}