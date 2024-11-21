#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "utils.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

void LoadData(const std::filesystem::path &baseDir,
              std::vector<std::string> &vstrImageFilenames,
              const std::string &sensor_dir, const std::string &file_extension);
std::filesystem::path findSubDir(const std::filesystem::path &baseDir,
                                 const std::string &targetDir);

std::filesystem::path findDataDir(const std::filesystem::path &baseDir,
                                  const std::string &sensor_dir);
int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << std::endl
              << "Usage: " << argv[0] << " path_to_data" /*1*/
              << " path_to_calib_dir"                    /*2*/
              << " path_to_out_dir"                      /*3*/
              << std::endl;
    return 1;
  }

  // Retrieve paths to images
  std::vector<std::string> vstrImageFilenamesRGB;
  std::vector<std::string> vstrImageFilenamesPCL;
  std::string strImageDir = std::string(argv[1]);
  std::filesystem::path pathImageDir(strImageDir);

  LoadData(pathImageDir, vstrImageFilenamesRGB, "image_02", ".png");
  LoadData(pathImageDir, vstrImageFilenamesPCL, "velodyne_points", ".bin");

  // Check consistency in the number of images
  if (vstrImageFilenamesRGB.empty()) {
    std::cerr << std::endl << "No images found in provided path." << std::endl;
    return 1;
  }

  // Check consistency in the number of images
  if (vstrImageFilenamesPCL.empty()) {
    std::cerr << std::endl << "No pcls found in provided path." << std::endl;
    return 1;
  }

  int nImages = vstrImageFilenamesRGB.size();
  int nPcls = vstrImageFilenamesPCL.size();
  if (vstrImageFilenamesPCL.size() != vstrImageFilenamesRGB.size()) {
    std::cerr << std::endl
              << "Different number of images and PCLs." << std::endl;
    return 1;
  }

  std::cout << std::endl << "-------" << std::endl;
  std::cout << "Start processing sequence ..." << std::endl;
  std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

  // Main loop
  cv::Mat imRGB;
  for (int ni = 0; ni < nImages; ni++) {
    // Read image
    imRGB = cv::imread(vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
    cv::cvtColor(imRGB, imRGB, cv::COLOR_BGR2RGB);

    // Image of zeros
    cv::Mat imD = cv::Mat::zeros(imRGB.size(), CV_16U);

    if (imRGB.empty()) {
      std::cerr << std::endl
                << "Failed to load image at: " << vstrImageFilenamesRGB[ni]
                << std::endl;
      return 1;
    }

    std::vector<LidarPoint> lidar_pts;
    readLidarPtsBin(vstrImageFilenamesPCL[ni].c_str(), lidar_pts);
    if (lidar_pts.empty()) {
      std::cerr << std::endl
                << "Failed to load PCL at: " << vstrImageFilenamesPCL[ni]
                << std::endl;
      return 1;
    }
    depthImage(imD, lidar_pts, argv[2]);
    std::string fileName = createFileName(ni);
    saveImage(imD, argv[3], fileName);
  }
}

void LoadData(const std::filesystem::path &baseDir,
              std::vector<std::string> &vstrImageFilenames,
              const std::string &sensor_dir,
              const std::string &file_extension) {

  std::filesystem::path dataDir = findDataDir(baseDir, sensor_dir);
  if (dataDir.empty()) {
    return; // Exit if `dataDir` is invalid
  }

  // Iterate through the contents of the `data` subdirectory
  for (const auto &imagePath : std::filesystem::directory_iterator(dataDir)) {
    if (imagePath.path().extension() != file_extension) {
      continue; // Skip files with the wrong extension
    }
    vstrImageFilenames.push_back(imagePath.path().string());
  }

  // Sort filenames if any were found
  if (!vstrImageFilenames.empty()) {
    std::sort(vstrImageFilenames.begin(), vstrImageFilenames.end());
  }
}

std::filesystem::path findDataDir(const std::filesystem::path &baseDir,
                                  const std::string &sensor_dir) {
  // Find the sensor directory
  std::filesystem::path sensorDir = findSubDir(baseDir, sensor_dir);
  if (sensorDir.empty()) {
    std::cerr << "Folder "
              << "'" << sensor_dir
              << "' not found in the base directory or its subdirectories."
              << std::endl;
    return {};
  }

  // Find the `data` subdirectory
  std::filesystem::path dataDir = sensorDir / "data";
  if (!std::filesystem::exists(dataDir) ||
      !std::filesystem::is_directory(dataDir)) {
    std::cerr << "'data' subdirectory not found in "
              << "'" << sensor_dir << "'" << std::endl;
    return {};
  }

  return dataDir;
}

std::filesystem::path findSubDir(const std::filesystem::path &baseDir,
                                 const std::string &targetDir) {
  for (const auto &dirEntry :
       std::filesystem::recursive_directory_iterator(baseDir)) {
    if (dirEntry.is_directory() && dirEntry.path().filename() == targetDir) {
      return dirEntry.path();
    }
  }
  return {}; // Return an empty path if not found
}
