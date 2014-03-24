#include "ground_station/map_maker.h"

MapMaker::MapMaker(QObject * parent) :
  QObject(parent) {

}

MapMaker::~MapMaker() {
  
}

void MapMaker::correctMarkerInfo(geometry_msgs::PoseArray markerInfo) {
  qDebug() << "in map maker" << thread();
}
