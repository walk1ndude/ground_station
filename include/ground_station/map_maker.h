#ifndef MAP_MAKERH
#define MAP_MAKERH

#include <QtCore/QObject>

class MapMaker : public QObject {
  Q_OBJECT
public:
  explicit MapMaker(QObject * parent = 0);
  ~MapMaker();

private:
  
};
  
#endif