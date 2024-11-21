#include "utils.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <string>
/* TEMPLATES */

template <typename T> void write_pod(std::ofstream &out, T &t) {
  out.write(reinterpret_cast<char *>(&t), sizeof(T));
}

template <typename T> void read_pod(std::ifstream &in, T &t) {
  in.read(reinterpret_cast<char *>(&t), sizeof(T));
}

template <typename T>
void read_pod_vector(std::ifstream &in, std::vector<T> &vect) {
  long size;

  read_pod(in, size);
  for (int i = 0; i < size; ++i) {
    T t;
    read_pod(in, t);
    vect.push_back(t);
  }
}

template <typename T>
void write_pod_vector(std::ofstream &out, std::vector<T> &vect) {
  long size = vect.size();
  write_pod<long>(out, size);
  for (auto it = vect.begin(); it != vect.end(); ++it) {
    write_pod<T>(out, *it);
  }
}

void writeLidarPts(std::vector<LidarPoint> &input, const char *fileName) {
  std::ofstream out(fileName);
  write_pod_vector(out, input);
  out.close();
}

void readLidarPts(const char *fileName, std::vector<LidarPoint> &output) {
  std::ifstream in(fileName);
  read_pod_vector(in, output);
}

void readLidarPtsBin(const char *fileName, std::vector<LidarPoint> &output) {
  // Open the binary file
  std::ifstream in(fileName, std::ios::binary);

  // Check if file is open
  if (!in.is_open()) {
    std::cerr << "[Fatal Error] Failed to open Lidar file." << std::endl;
    return;
  }

  // Read the binary data into a vector of LidarPoints
  LidarPoint point;
  while (in.read(reinterpret_cast<char *>(&point), sizeof(LidarPoint))) {
    output.push_back(point);
  }

  in.close();
}

/*
        @loadCalibrationData assigns calibration values to calibration matrices

*/

void loadCameraToLidarCalibration(const std::string &calibrationFilePath,
                                  cv::Mat &RT) {
  std::ifstream calibFile(calibrationFilePath);
  if (!calibFile.is_open()) {
    std::cerr << "[Error] Could not open calibration file: "
              << calibrationFilePath << std::endl;
    return;
  }

  std::string line;
  while (std::getline(calibFile, line)) {
    std::istringstream lineStream(line);
    std::string label;
    lineStream >> label;

    // Process R and T
    if (label == "R:") {
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          lineStream >> RT.at<double>(i, j);
    } else if (label == "T:") {
      for (int i = 0; i < 3; ++i)
        lineStream >> RT.at<double>(i, 3);

      // Add the last row [0, 0, 0, 1] for homogeneity
      RT.at<double>(3, 0) = 0.0;
      RT.at<double>(3, 1) = 0.0;
      RT.at<double>(3, 2) = 0.0;
      RT.at<double>(3, 3) = 1.0;
    }
  }

  calibFile.close();
}

void loadCameraToCamera(const std::string &calibrationFilePath, cv::Mat &RT) {
  std::ifstream calibFile(calibrationFilePath);
  if (!calibFile.is_open()) {
    std::cerr << "[Error] Could not open calibration file: "
              << calibrationFilePath << std::endl;
    return;
  }

  std::string line;
  while (std::getline(calibFile, line)) {
    std::istringstream lineStream(line);
    std::string label;
    lineStream >> label;

    double value;
    // Process R and T
    if (label == "R_02:") {
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          lineStream >> RT.at<double>(i, j);
    } else if (label == "T_02:") {
      for (int i = 0; i < 3; ++i) {
        lineStream >> value;
        // value = -value;
        RT.at<double>(i, 3) = value;
      }

      // Add the last row [0, 0, 0, 1] for homogeneity
      RT.at<double>(3, 0) = 0.0;
      RT.at<double>(3, 1) = 0.0;
      RT.at<double>(3, 2) = 0.0;
      RT.at<double>(3, 3) = 1.0;
    }
  }

  calibFile.close();
}

void loadCameraCalibration(const std::string &calibrationFilePath, cv::Mat &P,
                           cv::Mat &R_rect) {
  std::ifstream calibFile(calibrationFilePath);
  if (!calibFile.is_open()) {
    std::cerr << "[Error] Could not open calibration file: "
              << calibrationFilePath << std::endl;
    return;
  }

  std::string line;
  while (std::getline(calibFile, line)) {
    std::istringstream lineStream(line);
    std::string label;
    lineStream >> label;

    // Process P_rect_00 and R_rect_00
    if (label == "P_rect_02:") {
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
          lineStream >> P.at<double>(i, j);
    } else if (label == "R_rect_02:") {

      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          lineStream >> R_rect.at<double>(i, j);
    }
  }

  R_rect.at<double>(0, 3) = 0.0;
  R_rect.at<double>(1, 3) = 0.0;
  R_rect.at<double>(2, 3) = 0.0;

  R_rect.at<double>(3, 0) = 0.0;
  R_rect.at<double>(3, 1) = 0.0;
  R_rect.at<double>(3, 2) = 0.0;
  R_rect.at<double>(3, 3) = 1.0;
  calibFile.close();
}

void printCvMat(cv::Mat &mat) {
  // Print the matrix
  for (int i = 0; i < mat.rows; ++i) {
    for (int j = 0; j < mat.cols; ++j) {
      std::cout << mat.at<double>(i, j) << " ";
    }
    std::cout << std::endl;
  }
}

void lidarOnImage(const cv::Mat &img,
                  const std::vector<LidarPoint> &lidarPoints) {

  // store calibration data in OpenCV matrices
  cv::Mat P_rect_02(3, 4, cv::DataType<double>::type);
  cv::Mat R_rect_02(4, 4, cv::DataType<double>::type);
  cv::Mat RT(4, 4, cv::DataType<double>::type);
  cv::Mat RT_cam2(4, 4, cv::DataType<double>::type);

  std::string calib_velo_to_cam =
      "../52/2011_09_26_calib/2011_09_26/calib_velo_to_cam.txt";
  std::string calib_cam_to_cam =
      "../52/2011_09_26_calib/2011_09_26/calib_cam_to_cam.txt";

  loadCameraToLidarCalibration(calib_velo_to_cam, RT);
  loadCameraCalibration(calib_cam_to_cam, P_rect_02, R_rect_02);
  loadCameraToCamera(calib_cam_to_cam, RT_cam2);
  // printCvMat(P_rect_02);
  // printCvMat(R_rect_02);
  // printCvMat(RT_cam2);

  // project lidar points
  cv::Mat visImg = img.clone();
  cv::Mat overlay = visImg.clone();

  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {

    float maxX = 25.0, maxY = 6.0, minZ = -1.4;

    if (it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ ||
        it->r < 0.01) {

      continue;
    }

    X.at<double>(0, 0) = it->x;
    X.at<double>(1, 0) = it->y;
    X.at<double>(2, 0) = it->z;
    X.at<double>(3, 0) = 1;

    Y = P_rect_02 * R_rect_02 * -1 * RT_cam2 * RT * X;
    cv::Point pt;
    pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
    pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

    float val = it->x;
    float maxVal = 20.0;
    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
    cv::circle(overlay, pt, 3, cv::Scalar(0, green, red), -1);
  }

  float opacity = 0.6;
  cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

  std::string windowName = "LiDAR data on image overlay";

  cv::namedWindow(windowName, 3);
  cv::imshow(windowName, visImg);
  cv::waitKey(0);
}

void depthImage(cv::Mat &imgD, const std::vector<LidarPoint> &lidarPoints,
                std ::string calibDir) {

  // store calibration data in OpenCV matrices
  cv::Mat P_rect_02(3, 4, cv::DataType<double>::type);
  cv::Mat R_rect_02(4, 4, cv::DataType<double>::type);
  cv::Mat RT(4, 4, cv::DataType<double>::type);
  cv::Mat RT_cam2(4, 4, cv::DataType<double>::type);

  std::string calib_velo_to_cam = calibDir + "calib_velo_to_cam.txt";
  std::string calib_cam_to_cam = calibDir + "calib_cam_to_cam.txt";

  loadCameraToLidarCalibration(calib_velo_to_cam, RT);
  loadCameraCalibration(calib_cam_to_cam, P_rect_02, R_rect_02);
  loadCameraToCamera(calib_cam_to_cam, RT_cam2);

  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {

    float maxX = 25.0, maxY = 6.0, minZ = -1.4;

    if (it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ ||
        it->r < 0.01) {

      continue;
    }

    X.at<double>(0, 0) = it->x;
    X.at<double>(1, 0) = it->y;
    X.at<double>(2, 0) = it->z;
    X.at<double>(3, 0) = 1;

    Y = P_rect_02 * R_rect_02 * -1 * RT_cam2 * RT * X;
    cv::Point pt;
    pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
    pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

    float val = it->x;
    float maxVal = 20.0;
    int precision = 1 << 16;
    int intensity =
        std::min(precision, (int)(precision * abs((val - maxVal) / maxVal)));

    // Spread the pixel point a bit out for more dense image.
    cv::circle(imgD, pt, 3, cv::Scalar(intensity), -1);
  }
}

void saveImage(const cv::Mat &img, const std::string &out_dir,
               const std::string &name) {
  // cv::imwrite(out_dir / "loss.jpg", img);

  // Create the directory if it doesn't exist
  std::filesystem::path dir(out_dir);
  if (!std::filesystem::exists(dir)) {
    std::filesystem::create_directories(dir);
  }

  // Construct the full path to the image file
  std::filesystem::path image_path = dir / name;

  // Save the image using OpenCV's imwrite
  if (!cv::imwrite(image_path.string(), img)) {
    throw std::runtime_error("Failed to save image to " + image_path.string());
  }
}

std::string createFileName(int fileNumber) {
  // Create a zero-padded filename
  std::stringstream ss;
  ss << std::setw(10) << std::setfill('0') << fileNumber << ".png";
  std::string filename = ss.str();
  return filename;
}

void showImage(cv::Mat &img) {
  std::string windowName = "LiDAR data on image overlay";
  cv::namedWindow(windowName, 3);
  cv::imshow(windowName, img);
  cv::waitKey(0);
}