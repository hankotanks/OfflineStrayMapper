#include "StrayMapper.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <optional>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <QApplication>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/core/Odometry.h>
#include "clipp.h"
#include "MapBuilder.h"
#include "StrayCamera.h"

StrayMapper::StrayMapper(int argc, char* argv[]) {
    std::string outPathRaw;
    auto cli = (
        clipp::value("input file", this->dataPath_),
        clipp::option("--display").set(this->showUI_).doc("show a 3D scene visualization"),
        clipp::option("-o", "--out").doc("specify output directory") & clipp::value("output path", outPathRaw),
        clipp::option("--save-pts").set(this->savePointCloud_) & clipp::value("voxel size", this->voxelSize_),
        clipp::option("--keep-tmp").set(this->keepTempFiles_).doc("preserve temp files (speeds up next run)")
    );

    if(!(clipp::parse(argc, argv, cli))) std::cout << clipp::make_man_page(cli, argv[0]);

    this->outPath_ = outPathRaw.empty() ? std::nullopt : std::optional(outPathRaw);

    assert(std::filesystem::exists(dataPath));
}

void StrayMapper::run(int argc, char* argv[]) {
    ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

	StrayMapper cfg(argc, argv);

    pcl::PointCloud<pcl::PointXYZRGB> cloudTemp;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloudTemp.makeShared();

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
	params.insert(ParametersPair(Parameters::kOdomGuessMotion(), "true"));
	params.insert(ParametersPair(Parameters::kOdomStrategy(), "0"));

	std::cout << "Preparing data" << std::endl;
	
	StrayCamera camera(cfg.dataPath_, cfg.outPath_, !(cfg.keepTempFiles_));

	std::cout << "Finished preparing data" << std::endl;

	if(camera.init()) {
		Odometry* odom = Odometry::create();
		OdometryInfo info;

		Rtabmap rtabmap;
		rtabmap.init(params);

		QApplication app(argc, argv);
		MapBuilder mapBuilder;
		if(cfg.showUI_) {
			mapBuilder.show();
			QApplication::processEvents();
		}

		std::cout << "Starting SLAM" << std::endl;

		SensorData cameraData = camera.takeImage();
		int cameraIteration = 0, cameraInterval = camera.getFrameCount() / 20;
		int odometryIteration = 0;
		while(cameraData.isValid() && ((cfg.showUI_ && mapBuilder.isVisible()) || !(cfg.showUI_))) {
			if(++cameraIteration < camera.getFrameCount()) {
				Transform pose = odom->process(cameraData, &info);

				if(rtabmap.process(cameraData, pose)) {
					if(cfg.showUI_) mapBuilder.processStatistics(rtabmap.getStatistics());
					if(rtabmap.getLoopClosureId() > 0) printf("Loop closure detected!\n");
				}

				if(cfg.showUI_) mapBuilder.processOdometry(cameraData, pose, info);

				if(cfg.savePointCloud_) {
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = \
						util3d::cloudsRGBFromSensorData(cameraData)[0];
					(*cloud) += *util3d::transformPointCloud(temp, pose);
				}

				if(cameraIteration % cameraInterval == 0) {
					int completion = (int) ((float) cameraIteration / (float) camera.getFrameCount() * 100.f);
					std::cout << "[" << std::setfill('0') << std::setw(2) << completion;
					std::cout << "%] processed " << cameraIteration;
					std::cout << " out of " << camera.getFrameCount() << " frames." << std::endl;
				}
			}

			cameraData = camera.takeImage();

			if(cfg.showUI_) {
				QApplication::processEvents();
				while(mapBuilder.isPaused() && mapBuilder.isVisible()) {
					uSleep(100);
					QApplication::processEvents();
				}
			}
		}

		std::cout << "Completed SLAM" << std::endl;

		delete odom;

		if(cfg.showUI_ && mapBuilder.isVisible()) app.exec();

	} else UERROR("Camera init failed!");

	if(cfg.savePointCloud_) {
		util3d::voxelize(cloud, cfg.voxelSize_);

		std::string rootFmt(cfg.dataPath_);
		if(!rootFmt.empty() && rootFmt.back() == '/') rootFmt.pop_back();

		std::optional<std::string> outFmt(cfg.outPath_);
		if(outFmt && !(*outFmt).empty() && (*outFmt).back() == '/') (*outFmt).pop_back();
		
		std::string cloudPath = (outFmt ? (*outFmt) : rootFmt) + "/out.pcd";

		pcl::io::savePCDFile(cloudPath, *cloud);
	}
}
