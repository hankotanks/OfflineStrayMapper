#include "StrayMapper.h"
#include <cassert>
#include <cstring>
#include <iostream>
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
			clipp::required("sbf").set(saveSBF_) | 
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

	if(camera.init()) {
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
				if(savePCD_ || saveSBF_ || saveVTK_) {
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = \
						rtabmap::util3d::cloudsRGBFromSensorData(cameraData)[0];
					(*cloud) += *(rtabmap::util3d::transformPointCloud(temp, pose));
				}

				if(cameraIteration % cameraInterval == 0) {
					int completion = (int) ((float) cameraIteration / (float) camera.getFrameCount() * 100.f);
					UINFO("[%3d%] processed %d out of %d frames.", completion, cameraIteration, camera.getFrameCount());
				}
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

	} else UERROR("Camera init failed!");

	std::string rootFmt(dataPath_);
	if(!rootFmt.empty() && rootFmt.back() == '/') rootFmt.pop_back();

	std::optional<std::string> outFmt(outPath_);
	if(outFmt && !(*outFmt).empty() && (*outFmt).back() == '/') (*outFmt).pop_back();
	std::string cloudPath = (outFmt ? (*outFmt) : rootFmt) + "/out";
	if(savePCD_) {
		pcl::io::savePCDFile(cloudPath + ".pcd", *cloud);
	} else if(saveVTK_) {
		pcl::PCLPointCloud2 cloudVTKTemp;
		pcl::PCLPointCloud2::Ptr cloudVTK(&cloudVTKTemp);
		pcl::toPCLPointCloud2(*cloud, *cloudVTK);
		pcl::io::saveVTKFile(cloudPath + ".vtk", *cloudVTK);
	} else if(saveSBF_) {
		std::ofstream file(cloudPath + ".sbf", std::ios::binary);

		if(!file.is_open()) {
			UERROR("Failed to write SLAM result to %s.", (cloudPath + ".sbf").c_str());
			return 1;
		}

		char head[2] = {42, 42};
		file.write(head, 2);
		// 2
		uint64_t pt_count = static_cast<uint64_t>(cloud->size());
		file.write(reinterpret_cast<const char*>(&pt_count), sizeof(uint64_t));
		uint16_t sf_count = 3;
		file.write(reinterpret_cast<const char*>(&sf_count), sizeof(uint16_t));
		// 12
		double pt_shift = 0.0;
		file.write(reinterpret_cast<const char*>(&pt_shift), sizeof(double));
		file.write(reinterpret_cast<const char*>(&pt_shift), sizeof(double));
		file.write(reinterpret_cast<const char*>(&pt_shift), sizeof(double));
		// 12 + 24 = 36
		char pad[28] = {0};
		file.write(pad, 28);
		// 36 + 28 = 64
		float r, g, b;
		for (const auto& pt : cloud->points) {
			file.write(reinterpret_cast<const char*>(&(pt.x)), sizeof(float));
			file.write(reinterpret_cast<const char*>(&(pt.y)), sizeof(float));
			file.write(reinterpret_cast<const char*>(&(pt.z)), sizeof(float));
			r = ((float) pt.r) / 255.f;
			g = ((float) pt.g) / 255.f;
			b = ((float) pt.b) / 255.f;
			file.write(reinterpret_cast<const char*>(&r), sizeof(float));
			file.write(reinterpret_cast<const char*>(&g), sizeof(float));
			file.write(reinterpret_cast<const char*>(&b), sizeof(float));
		}

		file.close();
		
		// file = std::ofstream(cloudPath + ".sbf");
		// file << "[SBF]" << std::endl;
		// file << "Points=" << pt_count << std::endl;
		// file << "GlobalShift=0.0, 0.0, 0.0" << std::endl;
		// file << "SFCount=0" << std::endl;
		// file.close();
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