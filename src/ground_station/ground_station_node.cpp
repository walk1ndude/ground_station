#include <QtCore/QCoreApplication>

#include "ground_station/ground_station.h"

int main(int argc, char ** argv) {
  ros::init(argc, argv, "ground_station_node");
  
  QCoreApplication app(argc,argv);
  
  GroundStation groundStation;
  
  ros::Rate sleep_rate(50);
  
  while(ros::ok()) {
    ros::spinOnce();
    
    sleep_rate.sleep();
  }
  
  return app.exec();
}