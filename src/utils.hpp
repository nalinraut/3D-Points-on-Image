#ifndef UTILS_hpp
#define UTILS_hpp

#include "dataStructures.h"
#include <stdio.h>

/*
 @writeLidarPts writes Lidar data points to a file
*/
void writeLidarPts(std::vector<LidarPoint> &input, const char *fileName);

/*
 @readLidarPts reads Lidar data points from a file
*/
void readLidarPts(const char *fileName, std::vector<LidarPoint> &output);

void readLidarPtsBin(const char *fileName, std::vector<LidarPoint> &output);

/*
 @write_pod helps write each Lidar data points from a
                        LidarPoint struct to a vector that in
                        turn writes to a file
*/
template <typename T> void write_pod(std::ofstream &out, T &t);

/*
 @read_pod  helps read each Lidar data points from the
                        file to a struct LidarPoint which in
                        turn writes to a vector of LidarPoint(s)
*/
template <typename T> void read_pod(std::ifstream &in, T &t);

/*
 @read_pod_vector  reads each Lidar data points from the
                                   file to a struct LidarPoint which in
                               turn writes to a vector of LidarPoint(s)
*/
template <typename T>
void read_pod_vector(std::ifstream &in, std::vector<T> &vect);

/*
 @write_pod_vector  writes each Lidar data points from a
                                LidarPoint struct to a vector that in
                                turn writes to a file
*/
template <typename T>
void write_pod_vector(std::ofstream &out, std::vector<T> &vect);

void printCvMat(cv::Mat &mat);

void loadCameraCalibration(const std::string &calibrationFilePath, cv::Mat &P,
                           cv::Mat &R_rect);

void loadCameraToLidarCalibration(const std::string &calibrationFilePath,
                                  cv::Mat &RT);

void loadCameraToCamera(const std::string &calibrationFilePath, cv::Mat &RT);

void lidarOnImage(const cv::Mat &img,
                  const std::vector<LidarPoint> &lidarPoints);

void depthImage(cv::Mat &imgD, const std::vector<LidarPoint> &lidarPoints,
                std::string calibDir);

void saveImage(const cv::Mat &img, const std::string &out_dir,
               const std::string &name);

std::string createFileName(int fileNumber);

void showImage(cv::Mat &img);
#endif /* UTILS_hpp */