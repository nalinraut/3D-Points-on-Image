#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "utils.hpp"
#include <fstream>
#include <sstream>

using namespace std;

/*
        @lidarOnImage processes image and lidar points and projects the points
                                  on to the image

*/

int main(int argc, char const *argv[]) {

  cv::Mat img;
  vector<LidarPoint> lidar_pts;

  try {

    // load image in file
    // IMAGE PATH = "./images/0000000000.png"
    img = cv::imread(argv[1]);
    if (img.empty())
      throw(img);

    // load Lidar points from file
    // LIDAR_POINT_FILE = "./dat/C51_LidarPts_0000.dat"
    readLidarPtsBin(argv[2], lidar_pts);
    if (lidar_pts.empty())
      throw(lidar_pts);
  }

  catch (const cv::Mat &) {
    cerr << "[Fatal Error] Failed to load image. Check for Path." << endl;
  }

  catch (const std::vector<LidarPoint> &) {
    cerr << "[Fatal Error] Failed to load LiDAR points. Check for Path."
         << endl;
  }

  // Image of zeros
  cv::Mat imD = cv::Mat::zeros(img.size(), CV_16U);
  // lidarOnImage(img, lidar_pts);
  // depthImage(img, lidar_pts);
  // depthImage(imD, lidar_pts);
  // showImage(imD);
}