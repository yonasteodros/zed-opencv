#include "SaveDepth.hpp"

using namespace sl;
using namespace std;

int count_save = 0;
int mode_PointCloud = 0;
int mode_Depth = 0;
int PointCloud_format;
int Depth_format;

bool g_bFlag_reverse = false;
bool g_bFlag_TUM = false;
double g_dScale = 5000;//depth scale

std::string PointCloud_format_ext=".ply";
std::string Depth_format_ext=".png";


void setPointCloudFormatName(int format) {
    switch (format) {
        case 0:
        PointCloud_format_ext = ".xyz";
        break;
        case  1:
        PointCloud_format_ext = ".pcd";
        break;
        case  2:
        PointCloud_format_ext = ".ply";
        break;
        case  3:
        PointCloud_format_ext = ".vtk";
        break;
        default:
        break;
    }
}

void setDepthFormatName(int format) {
    switch (format) {
        case  0:
        Depth_format_ext = ".png";
        break;
        case  1:
        Depth_format_ext = ".pfm";
        break;
        case  2:
        Depth_format_ext = ".pgm";
        break;
        default:
        break;
    }
}

void processKeyEvent(Camera& zed, char &key) {
    switch (key) {
        case 'd':
        case 'D':
        saveDepth(zed, depthPath + prefixDepth + to_string(count_save));
        break;

        case 'b':
        case 'B':
        //saveRgbDepth(zed,count_save);
        break;

        case 'n': // Depth format
        case 'N':
        {
            mode_Depth++;
            Depth_format = (mode_Depth % 3);
            setDepthFormatName(Depth_format);
            std::cout << "Depth format: " << Depth_format_ext << std::endl;
        }
        break;

        case 'p':
        case 'P':
        savePointCloud(zed, path + prefixPointCloud + to_string(count_save));
        break;


        case 'm': // Point cloud format
        case 'M':
        {
            mode_PointCloud++;
            PointCloud_format = (mode_PointCloud % 4);
            setPointCloudFormatName(PointCloud_format);
            std::cout << "Point Cloud format: " << PointCloud_format_ext << std::endl;
        }
        break;

        case 'h': // Print help
        case 'H':
        cout << helpString << endl;
        break;

        case 's': // Save side by side image
        case 'S': 
        saveSbSImage(zed, std::string("ZED_image") + std::to_string(count_save) + std::string(".png"));
        break;
    }
    count_save++;
}

void savePointCloud(Camera& zed, std::string filename) {
    std::cout << "Saving Point Cloud... " << flush;

    sl::Mat point_cloud;
    zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

    auto state = point_cloud.write((filename + PointCloud_format_ext).c_str());

    if (state == ERROR_CODE::SUCCESS)
        std::cout << "Point Cloud has been saved under " << filename << PointCloud_format_ext << endl;
    else
        std::cout << "Failed to save point cloud... Please check that you have permissions to write at this location ("<< filename<<"). Re-run the sample with administrator rights under windows" << endl;
}

void saveDepth(Camera& zed, std::string filename) {
    std::cout << "Saving Depth Map... " << flush;

    sl::Mat depth;
    zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);

    convertUnit(depth, zed.getInitParameters().coordinate_units, UNIT::MILLIMETER);
    auto state = depth.write((filename + Depth_format_ext).c_str());

    if (state == ERROR_CODE::SUCCESS)
        std::cout << "Depth Map has been save under " << filename << Depth_format_ext << endl;
    else
		std::cout << "Failed to save depth map... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << endl;
}

void saveRgbDepth(Camera& zed, std::fstream& file_out) {
    std::cout << "Saving Depth Map... " << flush;

    sl::Mat depth_image_zed;
    sl::Mat image_sbs;

    zed.retrieveMeasure(depth_image_zed, sl::MEASURE::DEPTH);
    zed.retrieveImage(image_sbs, sl::VIEW::LEFT);
    //convertUnit(depth_image_zed, zed.getInitParameters().coordinate_units, UNIT::MILLIMETER);


    cv::Mat image_ocv = slMat2cvMat(image_sbs);
    cv::cvtColor(image_ocv, image_ocv, cv::COLOR_RGBA2BGR);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);

    Resolution image_size = zed.getCameraInformation().camera_configuration.resolution;
    int new_width = image_size.width;
    int new_height = image_size.height;

    cv::Size cvSize(new_width, new_height);
    cv::Mat depth(cvSize, CV_16UC1);
     depth_image_ocv *= 5000.0f;
     depth_image_ocv.convertTo(depth, CV_16UC1); // in mm, rounded

    auto timestamp = zed.getTimestamp(sl::TIME_REFERENCE::CURRENT).getMilliseconds();

    std::string depthFilename = depthPath + std::to_string(timestamp) + std::string(".png");
    std::string rgbFilename = rgbPath + std::to_string(timestamp) + std::string(".png");
    std::string depthName = "depth/" + std::to_string(timestamp) + std::string(".png");
    std::string rgbName = "rgb/" + std::to_string(timestamp) + std::string(".png");
    std::string assFile = std::to_string(timestamp) + " " + depthName +" " + std::to_string(timestamp) + " " + rgbName + "\n";

    //auto state = depth.write((depthFilename).c_str());
    auto state = cv::imwrite((depthFilename).c_str(), depth);

    if (!state)
        std::cout << "Depth Map has been save under " << depthFilename << endl;
    else
        std::cout << "Failed to save depth map... Please check that you have permissions to write at this location (" << depthFilename << "). Re-run the sample with administrator rights under windows" << endl;

    state = cv::imwrite((rgbFilename.c_str()), image_ocv);

    if (state)
        std::cout << "image has been save under " << rgbFilename << endl;
    else
        std::cout << "Failed to save image... Please check that you have permissions to write at this location (" << rgbFilename << "). Re-run the sample with administrator rights under windows" << endl;
    if(!file_out)
        std::cerr << "Could not open association file." << endl;
    file_out.write(assFile.data(), assFile.size());
}

void saveSbSImage(Camera& zed, std::string filename) {
    sl::Mat image_sbs;
    zed.retrieveImage(image_sbs, sl::VIEW::LEFT);

    auto state = image_sbs.write(filename.c_str());

	if (state == sl::ERROR_CODE::SUCCESS)
		std::cout << "Side by Side image has been save under " << filename << endl;
	else
		std::cout << "Failed to save image... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << endl;
}

/** @brief Convert png image files to .klg format
 *
 *  @param vec_info vector of path <timestamp, <depth path, rgb path>>
 *  @param strKlgFileName output file name
 *  @return void
 */

void convert(
    VEC_INFO &vec_info)
{
    std::cout << "klg_name:\n\t" << strKlgFileName << std::endl;


    std::string filename = strKlgFileName;//"test2.klg";
    FILE * logFile = fopen(filename.c_str(), "wb+");

    int32_t numFrames = (int32_t)vec_info.size();

    fwrite(&numFrames, sizeof(int32_t), 1, logFile);

    //CvMat *encodedImage = 0;

    VEC_INFO::iterator it = vec_info.begin();
    int count = 1;
    std::cout << "Progress:\n";
    for(it; it != vec_info.end(); it++)
    {
        std::string strAbsPathDepth =
            std::string(
                        getcwd(NULL, 0)) + "/" +
                        it->second.first;


        cv::Mat depth = imread(strAbsPathDepth.c_str(), cv::IMREAD_UNCHANGED);

        double depthScale = g_dScale;
        depth.convertTo(depth, CV_16UC1, 1000 * 1.0 / depthScale);

        int32_t depthSize = depth.total() * depth.elemSize();

        std::string strAbsPath = std::string(
                    getcwd(NULL, 0)) + "/" +
                    it->second.second;

        cv::Mat img =
            imread(strAbsPath.c_str(),
                        cv::IMREAD_UNCHANGED);
        /*if(img == NULL)
        {
            fclose(logFile);
            return;
        }*/

        int32_t imageSize = img.rows * img.cols * sizeof(unsigned char) * 3;

        unsigned char * rgbData = 0;
        rgbData = (unsigned char *)img.data;

        std::cout << '\r'
                  << std::setw(4) << std::setfill('0') << count << " / "
                  << std::setw(4) << std::setfill('0') << vec_info.size()
                  << std::flush;
        count++;

        /// Timestamp
        fwrite(&it->first, sizeof(int64_t), 1, logFile);

        /// DepthSize
        fwrite(&depthSize, sizeof(int32_t), 1, logFile);

        /// imageSize
        fwrite(&imageSize, sizeof(int32_t), 1, logFile);

        /// Depth buffer
        fwrite((char*)depth.data, depthSize, 1, logFile);

        /// RGB buffer
        fwrite(rgbData, imageSize, 1, logFile);

        //cv::ReleaseImage(&img);
        depth.release();
    }
    std::cout << std::endl;

    fclose(logFile);
}



/** @brief Parse associations.txt files to vector
 *
 *  @param strAssociation_Path path of associations.txt
 *  @param vec_info vector of path <timestamp, <depth path, rgb path>>
 *  @return void
 */
int parseInfoFile(
            std::string &strAssociation_Path,
            VEC_INFO &vec_info)
{
    char * line = NULL;
    size_t len = 0;
    ssize_t read;

    FILE *pFile = fopen(strAssociation_Path.c_str(), "r");
    if(!pFile) {
        return -1;
    }

    int iFrameCnt = 0;
    while ((read = getline(&line, &len, pFile)) != -1) {

        std::istringstream is(line);
        std::string part;
        int iIdxToken = 0;
        while (getline(is, part))
        {
            if('#' == part[0])/// Skip file comment '#"
            {
                continue;
            }

            int64_t timeSeq = 0;
            std::string strDepthPath;
            std::string strRgbPath;

            std::istringstream iss(part);
            std::string token;

            while (getline(iss, token, ' '))
            {
                if(2 == iIdxToken) //Time rgb
                {//first token which is time


                }
                else if(3 == iIdxToken)//rgb path
                {
                    strRgbPath = token;
                }
                else if(0 == iIdxToken)//Time depth
                {
                    /// Do nothing
                    //std::cout << token << std::endl;
                    token.erase(
                        std::remove(token.begin(),
                            token.end(), '.'), token.end());
                    //std::cout << token << std::endl;
                    long long unsigned int numb;
                    std::istringstream ( token ) >> numb;

                    if(true == g_bFlag_TUM)
                    {
                        timeSeq = (int64_t)numb;
                        //timeSeq = iFrameCnt;
                        iFrameCnt++;
                    }
                    else
                    {
                        timeSeq = numb * 1000000;
                    }
                }
                else if(1 == iIdxToken)//depth path
                {
                    strDepthPath = token;
                }
                iIdxToken++;
            }
            PATH_PAIR path_pair;
            if(g_bFlag_reverse)
                path_pair = PATH_PAIR(strRgbPath, strDepthPath);
            else
                path_pair = PATH_PAIR(strDepthPath, strRgbPath);
            SEQ seq(timeSeq, path_pair);
            vec_info.push_back(seq);
        }
    }
    fclose(pFile);
    return 0;
}

int convertToKlg(){

       int option_count = 0;
       std::string strAssociation_Path;

       int c = 0;

             /// Change working directory
       int ret = chdir(strWorkingDir.c_str());
       if(ret != 0)
       {
           fprintf(stderr, RED "dataset path not exist" NONE);
       }
       printf("\nCurrent working directory:\n\t%s\n", getcwd(NULL, 0));


       strAssociation_Path = strWorkingDir;
       if(strWorkingDir[strWorkingDir.length() - 1] != '/')
       {
           strAssociation_Path += '/';
       }
       strAssociation_Path += "associations.txt";

       /// Parse files
       // (timestamp, (depth path, rgb path) )
       VEC_INFO vec_info;


       int err = parseInfoFile(
                   strAssociation_Path,
                   vec_info);
       if(err != 0)
       {
           fprintf(stderr,
               RED "Fail to find associations.txt under working directory!\n" NONE);
           return -1;
       }

       std::cout << "Depth: " << vec_info.back().second.first << std::endl;
       std::cout << "RGB: " << vec_info.back().second.second << std::endl;
       std::cout << "scale: " << g_dScale << std::endl;

       convert(vec_info);

       std::cout << "Conversion complete!\n";
    return 0;
}


cv::Mat slMat2cvMat(sl::Mat &mat) {
    if (mat.getMemoryType() == sl::MEM::GPU)
        mat.updateCPUfromGPU();

    int cvType;
    switch (mat.getDataType()) {
        case sl::MAT_TYPE::F32_C1:
            cvType = CV_32FC1;
            break;
        case sl::MAT_TYPE::F32_C2:
            cvType = CV_32FC2;
            break;
        case sl::MAT_TYPE::F32_C3:
            cvType = CV_32FC3;
            break;
        case sl::MAT_TYPE::F32_C4:
            cvType = CV_32FC4;
            break;
        case sl::MAT_TYPE::U8_C1:
            cvType = CV_8UC1;
            break;
        case sl::MAT_TYPE::U8_C2:
            cvType = CV_8UC2;
            break;
        case sl::MAT_TYPE::U8_C3:
            cvType = CV_8UC3;
            break;
        case sl::MAT_TYPE::U8_C4:
            cvType = CV_8UC4;
            break;
    }
    return cv::Mat((int) mat.getHeight(), (int) mat.getWidth(), cvType, mat.getPtr<sl::uchar1>(sl::MEM::CPU), mat.getStepBytes(sl::MEM::CPU));
}
