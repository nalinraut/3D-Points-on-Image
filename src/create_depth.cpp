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
              std::string sensor_dir, std::string file_extension);

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
    depthImage(imD, lidar_pts);
    saveImage(imD, argv[3], std::to_string(ni));
  }
}

void LoadData(const std::filesystem::path &baseDir,
              std::vector<std::string> &vstrImageFilenames,
              std::string sensor_dir, std::string file_extension) {
  bool imageDirFound = false;

  for (const auto &dirEntry :
       std::filesystem::recursive_directory_iterator(baseDir)) {
    if (dirEntry.is_directory() && dirEntry.path().filename() == sensor_dir) {
      // Found the `image_00` directory, check for a `data` subdirectory
      std::filesystem::path dataDir = dirEntry.path() / "data";

      if (std::filesystem::exists(dataDir) &&
          std::filesystem::is_directory(dataDir)) {
        // Iterate through the contents of the `data` subdirectory
        for (const auto &imagePath :
             std::filesystem::directory_iterator(dataDir)) {
          // Check if the file has a `.png` extension
          if (imagePath.path().extension() == file_extension) {
            // std::cout << imagePath.path().string() << std::endl;
            vstrImageFilenames.push_back(imagePath.path().string());
          }
        }
      } else {
        std::cerr << "'data' subdirectory not found in "
                  << "'" << sensor_dir << "'" << std::endl;
      }

      // Stop searching after processing the first `image_00` directory
      break;
    }
  }

  std::sort(vstrImageFilenames.begin(), vstrImageFilenames.end());
  imageDirFound = true;

  if (!imageDirFound) {
    std::cerr << "Folder "
              << "'" << sensor_dir
              << " not found in the base directory or its "
                 "subdirectories."
              << std::endl;
  }
}
