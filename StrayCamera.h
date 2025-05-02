#ifndef CAMERA_WRAPPER_H
#define CAMERA_WRAPPER_H

#include <string>
#include <filesystem>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/camera/CameraRGBDImages.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/opencv.hpp>

#include "lazycsv.hpp"

namespace rtabmap {
    class CameraRGBDImagesWrapper : public rtabmap::CameraRGBDImages {
    public:
        CameraRGBDImagesWrapper(std::tuple<std::string, std::string, float> params) :
            rtabmap::CameraRGBDImages(
                std::get<0>(params), 
                std::get<1>(params), 
                std::get<2>(params)
            ) { /*  */ }
        virtual bool init(std::string& calibrationFolder, std::string& cameraName) {
            return rtabmap::CameraRGBDImages::init(calibrationFolder, cameraName);
        }
    };

    class StrayCamera : public CameraRGBDImagesWrapper {
    private:
        std::string root_;
        std::string calibrationFolder_;
        std::string cameraName_;
        std::vector<rtabmap::Transform> transforms_;
        unsigned int frameCount_;
    public:
        StrayCamera(const std::string& root) : CameraRGBDImagesWrapper(constructor(root)) {
            this->root_ = root;
            if (!this->root_.empty() && this->root_.back() == '/') this->root_.pop_back();
            const auto [width, height] = StrayCamera::imageDimensions(this->root_);
            const auto [calibrationFolder, cameraName] = StrayCamera::writeCameraCalibration(this->root_, width, height);
            this->calibrationFolder_ = calibrationFolder;
            this->cameraName_ = cameraName;
            this->transforms_ = StrayCamera::parseOdometry(root);
            // this->transforms_.pop_back();
            this->frameCount_ = StrayCamera::queryFrameCount(root);
        }

        ~StrayCamera() {
            const std::string temp = StrayCamera::pathOut(this->root_);

            if(std::filesystem::exists(temp)) std::filesystem::remove_all(temp);
        }

        virtual bool init() {
            return CameraRGBDImagesWrapper::init(this->calibrationFolder_, this->cameraName_);
        }

        unsigned int getFrameCount() {
            return this->frameCount_;
        }

        rtabmap::Transform getPose(int id) {
            return this->transforms_[id];
        }
    private:
        static std::string pathOut(const std::string& root) {
            return root + "/temp";
        }

        static std::string pathRGBImages(const std::string& root) {
            return StrayCamera::pathOut(root) + "/rgb";
        }

        static std::string pathDepthImages(const std::string& root) {
            return StrayCamera::pathOut(root) + "/depth";
        }

        static unsigned int queryFrameCount(const std::string& root) {
            cv::VideoCapture cap(root + "/rgb.mp4");
            // TODO: report error
            if(!cap.isOpened()) return 0;

            int frameCount = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
            if(frameCount < 0) return 0; // TODO: report error
            
            return (unsigned int) frameCount;
        }

        static unsigned int splitRGBImages(const std::string& root) {
            const std::string path = StrayCamera::pathRGBImages(root);
            std::filesystem::create_directories(path);
        
            cv::VideoCapture cap(root + "/rgb.mp4");
            if(!cap.isOpened()) return 0; // TODO: report error
        
            cv::Mat frame;
        
            int frameCount = 0;
            while(cap.read(frame)) {
                std::ostringstream filename;
                filename << path << "/" << std::setw(6) << std::setfill('0') << frameCount << ".png";
                cv::imwrite(filename.str(), frame);
                ++frameCount;
            }
        
            return frameCount;
        }

        static void copyDepthToOutput(const std::string& root, unsigned int frameCount) {
            const std::string path = pathDepthImages(root);
            std::filesystem::create_directories(path);
        
            for(auto i = 0; i < frameCount; ++i) {
                std::ostringstream src, dst;
                src << root << "/depth/" << std::setw(6) << std::setfill('0') << i << ".png";
                dst << path << "/" << std::setw(6) << std::setfill('0') << i << ".png";
                
                if(std::filesystem::is_regular_file(src.str())) {
                    std::filesystem::copy(src.str(), dst.str(), std::filesystem::copy_options::overwrite_existing);
                }
            }
        }

        static float scaleFactor(const std::string& root) {
            cv::Mat color = cv::imread(StrayCamera::pathRGBImages(root) + "/000000.png");
            cv::Mat depth = cv::imread(StrayCamera::pathDepthImages(root) + "/000000.png");
        
            const float x = (float) color.cols / (float) depth.cols;
            const float y = (float) color.rows / (float) depth.rows;
        
            assert(std::fabs(x - y) < 1e-6);
        
            return x;
        }

        static std::pair<unsigned int, unsigned int> imageDimensions(const std::string& root) {
            cv::Mat color = cv::imread(StrayCamera::pathRGBImages(root) + "/000000.png");

            unsigned int cols, rows;
            cols = (unsigned int) color.cols;
            rows = (unsigned int) color.rows;
        
            return std::pair<unsigned int, unsigned int>(cols, rows);
        }

        static std::pair<std::string, std::string> writeCameraCalibration(
            const std::string& root, 
            unsigned int width, 
            unsigned int height
        ) {
            const std::string calibrationFolder = StrayCamera::pathOut(root);
            const std::string cameraName = "iphone16pro";
        
            lazycsv::parser<lazycsv::mmap_source,
                lazycsv::has_header<false>,
                lazycsv::delimiter<','>,
                lazycsv::quote_char<'"'>,
                lazycsv::trim_chars<' ', '\t'>> parser { root + "/camera_matrix.csv" };
            float raw[3][3];
            int rowCount = 0;
            char* term;
            for(const auto row : parser) {
                const auto [fst, snd, thd] = row.cells(0, 1, 2);
                raw[rowCount][0] = std::strtof(fst.raw().data(), &term);
                raw[rowCount][1] = std::strtof(snd.raw().data(), &term);
                raw[rowCount][2] = std::strtof(thd.raw().data(), &term);
                ++rowCount;
            }
        
            std::ofstream file(calibrationFolder + "/" + cameraName + ".yaml");
            file << "%YAML:1.0" << std::endl;
            file << "---" << std::endl;
            file << "camera_name: iphone16pro" << std::endl;
            file << "image_width: " << width << std::endl;
            file << "image_height: " << height << std::endl;
            file << "camera_matrix:" << std::endl;
            file << "   rows: 3" << std::endl;
            file << "   cols: 3" << std::endl;
            file << "   data: [ ";
            file << raw[0][0] << ", " << raw[0][1] << ", " << raw[0][2] << ", ";
            file << raw[1][0] << ", " << raw[1][1] << ", " << raw[1][2] << ", ";
            file << raw[2][0] << ", " << raw[2][1] << ", " << raw[2][2] << " ]" << std::endl;
            file << "local_transform:" << std::endl;
            file << "   rows: 3" << std::endl;
            file << "   cols: 4" << std::endl;
            file << "   data: [  1.0,  0.0,  0.0, 0.0, " << std::endl;
            file << "            0.0,  1.0,  0.0, 0.0, " << std::endl;
            file << "            0.0,  0.0,  1.0, 0.0 ]" << std::endl;
            file << "distortion_coefficients:" << std::endl;
            file << "   rows: 1" << std::endl;
            file << "   cols: 5" << std::endl;
            file << "   data: [ 0., 0., 0., 0., 0. ]" << std::endl;
            file << "distortion_model: plumb_bob" << std::endl;
            file << "rectification_matrix:" << std::endl;
            file << "   rows: 3" << std::endl;
            file << "   cols: 3" << std::endl;
            file << "   data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]" << std::endl;
            file << "projection_matrix:" << std::endl;
            file << "   rows: 3" << std::endl;
            file << "   cols: 4" << std::endl;
            file << "   data: [ ";
            file << raw[0][0] << ", " << raw[0][1] << ", " << raw[0][2] << ", 0.0, ";
            file << raw[1][0] << ", " << raw[1][1] << ", " << raw[1][2] << ", 0.0, ";
            file << raw[2][0] << ", " << raw[2][1] << ", " << raw[2][2] << ", 0.0 ]" << std::endl;
            file.close();
        
            return std::pair<std::string, std::string>(calibrationFolder, cameraName);
        }

        static std::vector<rtabmap::Transform> parseOdometry(const std::string& root) {
            lazycsv::parser<> parser { root + "/odometry.csv" };
            std::vector<rtabmap::Transform> odom;
            int firstRow = 1;
            char* term;
            float x, y, z, qx, qy, qz, qw;
            for(const auto row : parser) {
                const auto raw = row.cells(2, 3, 4, 5, 6, 7, 8);
                float tx, ty, tz;
                tx = std::strtof(raw[0].raw().data(), &term);
                ty = std::strtof(raw[1].raw().data(), &term);
                tz = std::strtof(raw[2].raw().data(), &term);
                if(!firstRow) {
                    tx -= x;
                    x += tx;
                    ty -= y;
                    y += ty;
                    tz -= z;
                    z += tz;
                } else {
                    firstRow = 0;
                    x = tx;
                    y = ty;
                    z = tz;
                }
                qx = std::strtof(raw[3].raw().data(), &term);
                qy = std::strtof(raw[4].raw().data(), &term);
                qz = std::strtof(raw[5].raw().data(), &term);
                qw = std::strtof(raw[6].raw().data(), &term);
                odom.push_back(rtabmap::Transform(tx, ty, tz, qx, qy, qz, qw));
            }
        
            return odom;
        }

        static std::tuple<std::string, std::string, float> constructor(const std::string& root) {
            std::string rootFmt(root);
            if (!rootFmt.empty() && rootFmt.back() == '/') rootFmt.pop_back();
            
            const std::string temp = StrayCamera::pathOut(rootFmt);
            if(std::filesystem::exists(temp)) {
                std::filesystem::remove_all(temp);
                std::filesystem::create_directories(temp);
            }

            const unsigned int frameCount = StrayCamera::splitRGBImages(root);
            if(frameCount == 0) {
                UERROR("Failed to generate RGB images.");
                // TODO: Error handling
            }

            StrayCamera::copyDepthToOutput(root, frameCount);

            return std::tuple<std::string, std::string, float>(
                StrayCamera::pathRGBImages(root), 
                StrayCamera::pathDepthImages(root), 
                StrayCamera::scaleFactor(root)
            );
        }
    };
}

#endif // CAMERA_WRAPPER_H