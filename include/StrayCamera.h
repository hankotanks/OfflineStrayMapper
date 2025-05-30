#ifndef CAMERA_WRAPPER_H
#define CAMERA_WRAPPER_H

#include <optional>
#include <string>
#include <fstream>
#include <filesystem>
#include <limits>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/camera/CameraRGBDImages.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/opencv.hpp>
#include "lazycsv.h"
#ifdef WITH_PROMPT_DA
#include "util/PyScript.h"
#endif

namespace rtabmap {
    class CameraRGBDImagesWrapper : public CameraRGBDImages {
    public:
        CameraRGBDImagesWrapper(std::tuple<std::string, std::string, float> p) :
            CameraRGBDImages(std::get<0>(p), std::get<1>(p), std::get<2>(p)) { /*  */ }
        virtual bool init(std::string& calibrationFolder, std::string& cameraName) {
            return CameraRGBDImages::init(calibrationFolder, cameraName);
        }
    };

    class StrayCamera : public CameraRGBDImagesWrapper {
    private:
        bool removeTemp_;
        std::string root_, calibrationFolder_, cameraName_;
        std::optional<std::string> out_;
        unsigned int frameCount_;
        std::vector<float> stamps_;
        std::vector<Transform> groundTruthPoses_;
        std::vector<IMUEvent> sensorData_;
    public:
        StrayCamera(
            const std::string& root, 
            const std::optional<std::string>& out = {}, 
            const bool removeTemp = true,
            bool upscaleDepth = false
        ) : CameraRGBDImagesWrapper(constructor(root, out, upscaleDepth)) {
            this->removeTemp_ = removeTemp;
            
            // set stray root path
            this->root_ = root;
            if(!this->root_.empty() && this->root_.back() == '/') this->root_.pop_back();
            
            // set output path (if one was provided)
            this->out_ = out;
            if(this->out_ && !(*(this->out_)).empty() && (*(this->out_)).back() == '/') 
                (*(this->out_)).pop_back();
            
            // build camera calibration
            const auto [width, height] = StrayCamera::imageDimensions(this->root_, this->out_);
            const auto [calibrationFolder, cameraName] = StrayCamera::writeCameraCalibration(this->root_, this->out_, width, height);
            this->calibrationFolder_ = calibrationFolder;
            this->cameraName_ = cameraName;
            
            // set frame count
            this->frameCount_ = StrayCamera::queryFrameCount(root);

            // parse odometry (only for validation)
            const auto [stamps, groundTruthPoses] = StrayCamera::parseOdometry(root);
            this->stamps_ = stamps;
            this->groundTruthPoses_ = groundTruthPoses;
            this->sensorData_ = StrayCamera::parseIMU(root, this->stamps_);

            // configure timestamps
            const std::vector<float> stampsTemp(this->stamps_.begin(), this->stamps_.end() - 1);
            const std::string pathTimestamps = StrayCamera::writeTimestamps(this->root_, this->out_, stampsTemp);
            CameraImages::setTimestamps(false, pathTimestamps, false);
        }

        ~StrayCamera() {
            const std::string temp = StrayCamera::pathOut(this->root_, this->out_);

            if(std::filesystem::exists(temp) && this->removeTemp_) std::filesystem::remove_all(temp);
        }

        virtual bool init() {
            return CameraRGBDImagesWrapper::init(this->calibrationFolder_, this->cameraName_);
        }

        unsigned int getFrameCount() {
            return this->frameCount_;
        }

        Transform getPose(int id) {
            return this->groundTruthPoses_[id];
        }

        SensorData takeImage() {
            SensorData data = Camera::takeImage();
            if(data.isValid()) {
                IMUEvent event = this->sensorData_[data.id()];
                data.setIMU(event.getData());
            }

            return data;
        }

    private:
        static std::string pathOut(const std::string& root, const std::optional<std::string>& out) {
            return out ? (*out) + "/temp" : root + "/temp";
        }

        static std::string pathRGBImages(const std::string& root, const std::optional<std::string>& out) {
            return StrayCamera::pathOut(root, out) + "/rgb";
        }

        static std::string pathDepthImages(const std::string& root, const std::optional<std::string>& out) {
            return StrayCamera::pathOut(root, out) + "/depth";
        }

        static std::string pathDepthImagesUpscaled(const std::string& root, const std::optional<std::string>& out) {
            return StrayCamera::pathOut(root, out) + "/depth_upscaled";
        }

        static unsigned int queryFrameCount(const std::string& root) {
            cv::VideoCapture cap(root + "/rgb.mp4");
            // TODO: report error
            if(!cap.isOpened()) return 0;

            int frameCount = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
            if(frameCount < 0) return 0; // TODO: report error
            
            return (unsigned int) frameCount;
        }

        static unsigned int splitRGBImages(const std::string& root, const std::optional<std::string>& out) {
            const std::string path = StrayCamera::pathRGBImages(root, out);
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

        static void upscaleDepthImagery(
            const std::string& root,
            const std::optional<std::string>& out,
            bool& upscaleDepth
        ) {
            const std::string pathUpscaled = StrayCamera::pathDepthImagesUpscaled(root, out);
            std::filesystem::create_directories(pathUpscaled);
            
            unsigned int failed = 0;

            // temporary option as I don't have access to CUDA cores
            // mimics the results of PromptDA
#ifndef WITH_PROMPT_DA
            const auto [width, height] = StrayCamera::imageDimensions(root, out);
            const std::string pathDepth = StrayCamera::pathDepthImages(root, out);
            for(const auto& entry : std::filesystem::directory_iterator(pathDepth)) {
                if(entry.is_regular_file() && entry.path().extension() == ".png") {
                    cv::Mat depth = cv::imread(entry.path().string(), cv::IMREAD_UNCHANGED);
                    if(depth.empty()) {
                        failed = 1;
                        break;
                    }

                    cv::Size sizeUpscaled(width, height);
                    cv::Mat depthUpscaled;
                    cv::resize(depth, depthUpscaled, sizeUpscaled);
                    cv::imwrite(pathUpscaled + "/" + entry.path().filename().c_str(), depthUpscaled);
                }
            }

            if(failed) {
                UERROR("Error reading depth images from \"%s\".", pathDepth.c_str());
            }
#else
            PyScript upscaleDepthScript("upscale_depth");
            failed = upscaleDepthScript.call("main", "sss", 
                StrayCamera::pathRGBImages(root, out).c_str(),
                StrayCamera::pathDepthImages(root, out).c_str(),
                pathUpscaled.c_str()
            );
            
            if(failed) {
                UERROR("Failed to upscale depth imagery using PromptDA:");
                upscaleDepthScript.printErr();
            }
#endif

            if(failed) {
                UINFO("Proceeding with low-resolution depth imagery.");
                std::filesystem::remove_all(pathUpscaled);
                upscaleDepth = false;
            }
        }

        static void copyDepthToOutput(
            const std::string& root, 
            const std::optional<std::string>& out, 
            unsigned int frameCount
        ) {
            const std::string path = StrayCamera::pathDepthImages(root, out);
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

        static float scaleFactor(const std::string& root, const std::optional<std::string>& out) {
            cv::Mat color = cv::imread(StrayCamera::pathRGBImages(root, out) + "/000000.png");
            cv::Mat depth = cv::imread(StrayCamera::pathDepthImages(root, out) + "/000000.png");
        
            const float x = (float) color.cols / (float) depth.cols;
            const float y = (float) color.rows / (float) depth.rows;
        
            assert(std::fabs(x - y) < 1e-6);
        
            return x;
        }

        static std::pair<unsigned int, unsigned int> imageDimensions(
            const std::string& root, 
            const std::optional<std::string>& out
        ) {
            cv::Mat color = cv::imread(StrayCamera::pathRGBImages(root, out) + "/000000.png");

            unsigned int cols, rows;
            cols = (unsigned int) color.cols;
            rows = (unsigned int) color.rows;
        
            return std::pair<unsigned int, unsigned int>(cols, rows);
        }

        static std::pair<std::string, std::string> writeCameraCalibration(
            const std::string& root, 
            const std::optional<std::string>& out,
            unsigned int width, 
            unsigned int height
        ) {
            const std::string calibrationFolder = StrayCamera::pathOut(root, out);
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

        static std::pair<std::vector<float>, std::vector<Transform>> parseOdometry(const std::string& root) {
            lazycsv::parser<> parser { root + "/odometry.csv" };
            std::vector<Transform> odom;
            std::vector<float> stamps;
            char* term;
            float tx, ty, tz, qx, qy, qz, qw; // curr
            for(const auto row : parser) {
                const auto raw = row.cells(0, 2, 3, 4, 5, 6, 7, 8);
                stamps.push_back(std::strtof(raw[0].raw().data(), &term));
                tx = std::strtof(raw[1].raw().data(), &term);
                ty = std::strtof(raw[2].raw().data(), &term);
                tz = std::strtof(raw[3].raw().data(), &term);
                qx = std::strtof(raw[4].raw().data(), &term);
                qy = std::strtof(raw[5].raw().data(), &term);
                qz = std::strtof(raw[6].raw().data(), &term);
                qw = std::strtof(raw[7].raw().data(), &term);
                odom.push_back(Transform(tx, ty, tz, qx, qy, qz, qw));
            }
        
            return std::pair<std::vector<float>, std::vector<Transform>>(stamps, odom);
        }

        static std::vector<IMUEvent> parseIMU(const std::string& root, const std::vector<float>& stamps) {
            lazycsv::parser<> parser { root + "/imu.csv" };
            std::vector<IMUEvent> sensorData;

            int stampIdx = 0;
            float ts, lx, ly, lz, ax, ay, az;
            float t0 = std::numeric_limits<float>::max() * -1.f;
            char* term;
            for(const auto row : parser) {
                if(stampIdx >= stamps.size()) break;
                const auto raw = row.cells(0, 1, 2, 3, 4, 5, 6);
                ts = std::strtof(raw[0].raw().data(), &term);
                if(stamps[stampIdx] < ts) {
                    lx = std::strtof(raw[1].raw().data(), &term);
                    ly = std::strtof(raw[2].raw().data(), &term);
                    lz = std::strtof(raw[3].raw().data(), &term);
                    ax = std::strtof(raw[4].raw().data(), &term);
                    ay = std::strtof(raw[5].raw().data(), &term);
                    az = std::strtof(raw[6].raw().data(), &term);
                    IMU temp(
                        cv::Vec3d(ax, ay, az),
                        cv::Mat::eye(3, 3, CV_64F),
                        cv::Vec3d(lx, ly, lz),
                        cv::Mat::eye(3, 3, CV_64F)
                    );
                    sensorData.push_back(IMUEvent(temp, (double) t0));
                    stampIdx++;
                }
                t0 = ts;
            }

            return sensorData;
        }

        static std::string writeTimestamps(
            const std::string& root, 
            const std::optional<std::string>& out,
            const std::vector<float>& stamps
        ) {
            const std::string pathTimestamps = StrayCamera::pathOut(root, out) + "/timestamps.txt";
            std::ofstream outStamps(pathTimestamps);

            for(auto i = 0; i < stamps.size(); ++i) 
                outStamps << std::fixed << std::setprecision(6) << stamps[i] << std::endl;

            outStamps.close();

            return pathTimestamps;
        }

        static std::tuple<std::string, std::string, float> constructor(
            const std::string& root,
            const std::optional<std::string>& out,
            bool& upscaleDepth
        ) {
            std::string rootFmt(root);
            if(!rootFmt.empty() && rootFmt.back() == '/') rootFmt.pop_back();

            std::optional<std::string> outFmt(out);
            if(outFmt && !(*outFmt).empty() && (*outFmt).back() == '/') (*outFmt).pop_back();
            
            const std::string temp = StrayCamera::pathOut(rootFmt, outFmt);
            if(!std::filesystem::exists(temp)) {
                std::filesystem::create_directories(temp);

                const unsigned int frameCount = StrayCamera::splitRGBImages(rootFmt, outFmt);
                // this is a nice trick to allow logging before the error condition
                for(; frameCount == 0; assert(frameCount != 0)) {
                    UERROR("Stray data contained invalid RGB imagery.");
                }
                
                StrayCamera::copyDepthToOutput(rootFmt, outFmt, frameCount);
                if(upscaleDepth) StrayCamera::upscaleDepthImagery(rootFmt, outFmt, upscaleDepth);
            } else if(upscaleDepth && !std::filesystem::exists(StrayCamera::pathDepthImagesUpscaled(rootFmt, outFmt))){
                // the --upscale flag is the only flag that causes a change
                // in the /temp folder
                // as such, it is the only thing that can get out of sync between temp runs
                // depending on which flags are set
                StrayCamera::upscaleDepthImagery(rootFmt, outFmt, upscaleDepth);
            }

            // at this point, upscaleDepth is set false if PromptDA failed,
            // so we can use it to adjust the parameters to CameraRGBDImages
            return std::tuple<std::string, std::string, float>(
                StrayCamera::pathRGBImages(rootFmt, outFmt), 
                upscaleDepth 
                    ? StrayCamera::pathDepthImagesUpscaled(rootFmt, outFmt) 
                    : StrayCamera::pathDepthImages(rootFmt, outFmt), 
                upscaleDepth ? 1.f : StrayCamera::scaleFactor(rootFmt, outFmt)
            );
        }
    };
}

#endif // CAMERA_WRAPPER_H