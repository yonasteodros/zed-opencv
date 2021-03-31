#ifndef __SAVE_DEPTH_HPP__
#define __SAVE_DEPTH_HPP__

#define NOMINMAX

#include <iomanip>
#include <signal.h>
#include <iostream>
#include <limits>
#include <thread>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

const std::string helpString = "[d] Save Depth, [n] Change Depth format, [p] Save Point Cloud, [m] Change Point Cloud format, [q] Quit";
const std::string prefixPointCloud = "Cloud_"; // Default PointCloud output file prefix
const std::string prefixDepth = "Depth_"; // Default Depth image output file prefix
const std::string depthPath = "/media/user/Data/programs/zed-opencv/cpp/build/Dataset/depth/";
const std::string rgbPath = "/media/user/Data/programs/zed-opencv/cpp/build/Dataset/rgb/";
const std::string associationPath = "/media/user/Data/programs/zed-opencv/cpp/build/Dataset/associations.txt";
const std::string path = "/";




void savePointCloud(sl::Camera& zed, std::string filename);
void saveDepth(sl::Camera& zed, std::string filename);
void saveRgbDepth(sl::Camera& zed, std::fstream &file_out);
void saveSbSImage(sl::Camera& zed, std::string filename);

void processKeyEvent(sl::Camera& zed, char &key);

#endif
