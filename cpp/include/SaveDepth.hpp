#ifndef __SAVE_DEPTH_HPP__
#define __SAVE_DEPTH_HPP__

#define NOMINMAX

#include <iomanip>
#include <signal.h>
#include <iostream>
#include <limits>
#include <thread>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <utility>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#define NONE "\033[m"
#define RED "\033[0;32;31m"

extern char *optarg;
extern int optind;

/// PATH_PAIR = pair<depth image path, rgb image path>
typedef std::pair<std::string,std::string> PATH_PAIR;

/// SEQ = pair<timestamp, PATH_PAIR>
typedef std::pair<int64_t,PATH_PAIR> SEQ;
typedef std::vector<SEQ> VEC_INFO;

const std::string helpString = "[d] Save Depth, [n] Change Depth format, [p] Save Point Cloud, [m] Change Point Cloud format, [q] Quit";
const std::string prefixPointCloud = "Cloud_"; // Default PointCloud output file prefix
const std::string prefixDepth = "Depth_"; // Default Depth image output file prefix
const std::string depthPath = "/media/user/Data/programs/zed-opencv/cpp/build/Dataset/depth/";
const std::string rgbPath = "/media/user/Data/programs/zed-opencv/cpp/build/Dataset/rgb/";
const std::string associationPath = "/media/user/Data/programs/zed-opencv/cpp/build/Dataset/associations.txt";
const std::string strWorkingDir = "/media/user/Data/programs/zed-opencv/cpp/build/Dataset/";
const std::string strKlgFileName = "/media/user/Data/programs/zed-opencv/cpp/build/Dataset/data.klg" ;
const std::string path = "/";


void savePointCloud(sl::Camera& zed, std::string filename);
void saveDepth(sl::Camera& zed, std::string filename);
void saveRgbDepth(sl::Camera& zed, std::fstream &file_out);
void saveSbSImage(sl::Camera& zed, std::string filename);
void convert(VEC_INFO &vec_info);
cv::Mat slMat2cvMat(sl::Mat &mat);
int  convertToKlg();
int  parseInfoFile(std::string &strAssociation_Path,VEC_INFO &vec_info);
void convert();

void processKeyEvent(sl::Camera& zed, char &key);

#endif
