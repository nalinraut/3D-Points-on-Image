#ifndef dataStructures_h
#define dataStructures_h

#include <opencv2/core.hpp>
#include <vector>

struct LidarPoint { // single lidar point in space
  float x, y, z, r; // x,y,z in [m], r is point reflectivity
};

#endif /* dataStructures_h */
