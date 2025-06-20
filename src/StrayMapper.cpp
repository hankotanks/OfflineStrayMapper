#include "StrayMapper.h"
#include "rtabmap/core/Transform.h"
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <optional>
#include <filesystem>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#ifdef WITH_QT
#include <QApplication>
#endif
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/core/Odometry.h>
#include "clipp.h"
#ifdef WITH_QT
#include "MapBuilder.h"
#endif
#include "StrayCamera.h"

StrayMapper::StrayMapper(int argc, char* argv[]) : appName_(std::string(argv[0])) {
    std::string outPathRaw;
	std::string outCloudFormat = "";
    auto cli = (
        clipp::value("input folder", dataPath_),
		clipp::option("-o", "--out").doc("specifies output directory\nif not set, outputs to input folder") & 
			clipp::value("path", outPathRaw),
		clipp::option("-k", "--keep").set(keepTempFiles_).doc("preserve temp files (speeds up subsequent runs)"),
		clipp::option("--save").doc("save unified point cloud in the given format") & (
			clipp::required("pcd").set(savePCD_) |
			clipp::required("vtk").set(saveVTK_) ),
#ifdef WITH_PROMPT_DA
		clipp::option("-u", "--upscale").set(upscaleDepth_).doc("upscale depth imagery with PromptDA"),
#else
		clipp::option("-u", "--upscale").set(upscaleDepth_).doc("upscale depth imagery directly (WITH_PROMPT_DA=OFF)"),
#endif
#ifdef WITH_QT
        clipp::option("-v", "--visualize").set(showUI_).doc("show a 3D scene visualization"),
#endif
		clipp::option("--info").set(showInfo_).doc("log additional info to stdout (noisy)")
    );

    if(!(clipp::parse(argc, argv, cli))) {
		std::cout << clipp::make_man_page(cli, argv[0]);
	}

    outPath_ = outPathRaw.empty() ? std::nullopt : std::optional(outPathRaw);
}

static int STRAY_MAPPER_QAPPLICATION_ARGC = 1;

unsigned int StrayMapper::run() const {
    ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(showInfo_ ? ULogger::kInfo : ULogger::kError);

	UINFO("Checking provided paths.");
	if(StrayMapper::validate()) return 1;

    pcl::PointCloud<pcl::PointXYZRGB> cloudTemp;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloudTemp.makeShared();

	rtabmap::ParametersMap params;
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomGuessMotion(), "true"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomStrategy(), "0"));

	UINFO("Processing Stray data.");
	rtabmap::StrayCamera camera(dataPath_, outPath_, !(keepTempFiles_), upscaleDepth_);
	UINFO("Finished processing Stray data.");

	std::string rootFmt(dataPath_);
	if(!rootFmt.empty() && rootFmt.back() == '/') rootFmt.pop_back();

	std::optional<std::string> outFmt(outPath_);
	if(outFmt && !(*outFmt).empty() && (*outFmt).back() == '/') (*outFmt).pop_back();
	std::string outPath = (outFmt ? (*outFmt) : rootFmt);

	if(camera.init()) {
		std::vector<std::pair<double, rtabmap::Transform>> poses;

		rtabmap::Odometry* odom = rtabmap::Odometry::create();
		rtabmap::OdometryInfo info;

		rtabmap::Rtabmap rtabmap;
		rtabmap.init(params);

#ifdef WITH_QT
		char* appName = (char*) appName_.c_str();
		QApplication app(STRAY_MAPPER_QAPPLICATION_ARGC, &appName);
		MapBuilder mapBuilder;
		if(showUI_) {
			mapBuilder.show();
			QApplication::processEvents();
		}
#endif

		UINFO("Starting SLAM, processing frames.");

		rtabmap::SensorData cameraData = camera.takeImage();
		int cameraIteration = 0;
		int cameraInterval = camera.getFrameCount() / 20;
#ifdef WITH_QT
		while(cameraData.isValid() && ((showUI_ && mapBuilder.isVisible()) || !(showUI_))) {
#else
		while(cameraData.isValid()) {
#endif
			if(++cameraIteration < camera.getFrameCount()) {
				rtabmap::Transform pose = odom->process(cameraData, &info);

				if(rtabmap.process(cameraData, pose)) {
#ifdef WITH_QT
					if(showUI_) mapBuilder.processStatistics(rtabmap.getStatistics());
#endif
					if(rtabmap.getLoopClosureId() > 0) printf("Loop closure detected!\n");
				}
#ifdef WITH_QT
				if(showUI_) mapBuilder.processOdometry(cameraData, pose, info);
#endif
				if(savePCD_ || saveVTK_) {
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = \
						rtabmap::util3d::cloudsRGBFromSensorData(cameraData)[0];
					(*cloud) += *(rtabmap::util3d::transformPointCloud(temp, pose));
				}

				if(cameraIteration % cameraInterval == 0) {
					int completion = (int) ((float) cameraIteration / (float) camera.getFrameCount() * 100.f);
					UINFO("[%3d%] processed %d out of %d frames.", completion, cameraIteration, camera.getFrameCount());
				}

				poses.push_back(std::make_pair(cameraData.stamp(), odom->getPose()));
			}

			cameraData.clearRawData();
			cameraData.clearCompressedData();
			cameraData = camera.takeImage();

#ifdef WITH_QT
			if(showUI_) {
				QApplication::processEvents();
				while(mapBuilder.isPaused() && mapBuilder.isVisible()) {
					uSleep(100);
					QApplication::processEvents();
				}
			}
#endif
		}

		cameraData.clearRawData();
		cameraData.clearCompressedData();

		UINFO("Completed processing frames.");

		delete odom;

#ifdef WITH_QT
		if(showUI_ && mapBuilder.isVisible()) app.exec();
#endif

		// save odometry
		UINFO("Saving odometry (%s).", (outPath + "/poses.csv").c_str());

		std::ofstream poseFile(outPath + "/poses.csv");
		poseFile << "timestamp, frame, x, y, z, qx, qy, qz, qw" << std::endl;
		
		size_t poseIdx = 0;
		for(const auto& [stamp, pose] : poses) {
			auto t = pose.translation();
			auto q = pose.getQuaterniond();
			poseFile << std::setprecision(8) << std::fixed << stamp << ", ";
			poseFile << std::setfill('0') << std::setw(6) << poseIdx << ", ";
			poseFile << std::setprecision(9) << std::fixed << t.x() << ", ";
			poseFile << std::setprecision(9) << std::fixed << t.y() << ", ";
			poseFile << std::setprecision(9) << std::fixed << t.z() << ", ";
			poseFile << std::setprecision(9) << std::fixed << q.x() << ", ";
			poseFile << std::setprecision(9) << std::fixed << q.y() << ", ";
			poseFile << std::setprecision(9) << std::fixed << q.z() << ", ";
			poseFile << std::setprecision(9) << std::fixed << q.w() << ", " << std::endl;
			poseIdx++;
		}

		poseFile.close();

		UINFO("Finished saving odometry.");

	} else UERROR("Camera init failed!");

	if(savePCD_) {
		pcl::io::savePCDFileBinary(outPath + "/out.pcd", *cloud);
	} else if(saveVTK_) {
		pcl::PCLPointCloud2 cloudVTKTemp;
		pcl::PCLPointCloud2::Ptr cloudVTK(&cloudVTKTemp);
		pcl::toPCLPointCloud2(*cloud, *cloudVTK);
		pcl::io::saveVTKFile(outPath + "/out.vtk", *cloudVTK);
	}

	return 0;
}

unsigned int StrayMapper::validate() const {
	if(dataPath_.empty()) return 1;

	if(!std::filesystem::exists(dataPath_)) {
		UERROR("Unable to find provided Stray data (%s).", dataPath_.c_str());
		return 1;
	}

// #ifdef WITH_PROMPT_DA
#if 0
	if(upscaleDepth_) {
		PyScript checkCUDAScript("check_cuda_availability");
		if(checkCUDAScript.call("main", "")) {
			UERROR("System is not CUDA-compatible.");
			return 1;
		}
	}
#endif

	return 0;
}