#include "StrayMapper.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <optional>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <QApplication>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/core/Odometry.h>
#include "clipp.h"
#include "MapBuilder.h"
#include "StrayCamera.h"

StrayMapper::StrayMapper(int argc, char* argv[]) : appName_(std::string(argv[0])) {
    std::string outPathRaw;
    auto cli = (
        clipp::value("input folder", dataPath_),
		clipp::option("-o", "--out").doc("specifies output directory\nif not set, outputs to input folder") & 
			clipp::value("path", outPathRaw),
		clipp::option("--preserve").set(keepTempFiles_).doc("preserve temp files (speeds up next run)"),
		clipp::option("--pcd").set(savePointCloud_).doc("save unified point cloud to output folder"),
		clipp::option("-u", "--upscale").set(upscaleDepth_).doc("upscale depth imagery with PromptDA"),
        clipp::option("-v", "--visualize").set(showUI_).doc("show a 3D scene visualization"),
		clipp::option("--info").set(showInfo_).doc("log info to stdout")
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

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
	params.insert(ParametersPair(Parameters::kOdomGuessMotion(), "true"));
	params.insert(ParametersPair(Parameters::kOdomStrategy(), "0"));

	UINFO("Processing Stray data.");
	StrayCamera camera(dataPath_, outPath_, !(keepTempFiles_), upscaleDepth_);
	UINFO("Finished processing Stray data.");

	if(camera.init()) {
		Odometry* odom = Odometry::create();
		OdometryInfo info;

		Rtabmap rtabmap;
		rtabmap.init(params);

		char* appName = (char*) appName_.c_str();
		QApplication app(STRAY_MAPPER_QAPPLICATION_ARGC, &appName);
		MapBuilder mapBuilder;
		if(showUI_) {
			mapBuilder.show();
			QApplication::processEvents();
		}

		UINFO("Starting SLAM, processing frames.");

		SensorData cameraData = camera.takeImage();
		int cameraIteration = 0, cameraInterval = camera.getFrameCount() / 20;
		int odometryIteration = 0;
		while(cameraData.isValid() && ((showUI_ && mapBuilder.isVisible()) || !(showUI_))) {
			if(++cameraIteration < camera.getFrameCount()) {
				Transform pose = odom->process(cameraData, &info);

				if(rtabmap.process(cameraData, pose)) {
					if(showUI_) mapBuilder.processStatistics(rtabmap.getStatistics());
					if(rtabmap.getLoopClosureId() > 0) printf("Loop closure detected!\n");
				}

				if(showUI_) mapBuilder.processOdometry(cameraData, pose, info);

				if(savePointCloud_) {
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = \
						util3d::cloudsRGBFromSensorData(cameraData)[0];
					(*cloud) += *util3d::transformPointCloud(temp, pose);
				}

				if(cameraIteration % cameraInterval == 0) {
					int completion = (int) ((float) cameraIteration / (float) camera.getFrameCount() * 100.f);
					UINFO("[%3d%] processed %d out of %d frames.", completion, cameraIteration, camera.getFrameCount());
				}
			}

			cameraData = camera.takeImage();

			if(showUI_) {
				QApplication::processEvents();
				while(mapBuilder.isPaused() && mapBuilder.isVisible()) {
					uSleep(100);
					QApplication::processEvents();
				}
			}
		}

		UINFO("Completed processing frames.");

		delete odom;

		if(showUI_ && mapBuilder.isVisible()) app.exec();

	} else UERROR("Camera init failed!");

	if(savePointCloud_) {
		std::string rootFmt(dataPath_);
		if(!rootFmt.empty() && rootFmt.back() == '/') rootFmt.pop_back();

		std::optional<std::string> outFmt(outPath_);
		if(outFmt && !(*outFmt).empty() && (*outFmt).back() == '/') (*outFmt).pop_back();
		
		std::string cloudPath = (outFmt ? (*outFmt) : rootFmt) + "/out.pcd";
		// util3d::voxelize(cloud, voxelSize_);
		pcl::io::savePCDFile(cloudPath, *cloud);
	}

	return 0;
}

unsigned int StrayMapper::validate() const {
	if(dataPath_.empty()) return 1;

	if(!std::filesystem::exists(dataPath_)) {
		UERROR("Unable to find provided Stray data (%s)", dataPath_.c_str());
		return 1;
	}

	PyScript checkCUDAScript("check_cuda_availability");
	if(checkCUDAScript.call("main", "")) {
		UERROR("System is not CUDA-compatible.");
		return 1;
	}

	return 0;
}