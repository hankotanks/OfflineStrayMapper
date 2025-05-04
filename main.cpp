
// #define SAVE_POINT_CLOUD
// #define POINT_CLOUD_VOXEL_SIZE 0.5f
// #define POINT_CLOUD_OUT_PATH "../out.pcd"

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/utilite/UThread.h>
#ifdef SAVE_POINT_CLOUD
#include <pcl/io/pcd_io.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#endif
#include <QApplication>
#include "MapBuilder.h"
#include "StrayCamera.h"

using namespace rtabmap;

int main(int argc, char * argv[]) {
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

	if (argc != 2) {
		UERROR("Must provide path to Stray data");
		return 1;
	}

#ifdef SAVE_POINT_CLOUD
	pcl::PointCloud<pcl::PointXYZRGB> cloudTemp;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloudTemp.makeShared();
#endif

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "true"));
	params.insert(ParametersPair(Parameters::kOdomGuessMotion(), "true"));
	params.insert(ParametersPair(Parameters::kOdomStrategy(), "0"));
	
	StrayCamera camera(argv[1], false);

	if(camera.init()) {
		Rtabmap rtabmap;
		rtabmap.init(params);

		QApplication app(argc, argv);
		MapBuilder mapBuilder;
		mapBuilder.show();
		QApplication::processEvents();

		SensorData cameraData = camera.takeImage();
		SensorData sensorData;
		int cameraIteration = 0;
		int sensorIteration = 0;
		while(cameraData.isValid() && mapBuilder.isVisible()) {
			if(++cameraIteration < camera.getFrameCount()) {
				Transform pose = camera.getPose(cameraData.id());

				for(; camera.getIMU(sensorIteration).getStamp() < cameraData.stamp(); ++sensorIteration) {
					IMUEvent sensorEvent = camera.getIMU(sensorIteration);
					sensorData = SensorData(sensorEvent.getData(), sensorEvent.getStamp());
					rtabmap.process(sensorData, pose);
				}

				if(rtabmap.process(cameraData, pose)) {
					mapBuilder.processStatistics(rtabmap.getStatistics());
					if(rtabmap.getLoopClosureId() > 0) {
						printf("Loop closure detected!\n");
					}
				}

#ifdef SAVE_POINT_CLOUD
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = util3d::cloudsRGBFromSensorData(cameraData)[0];
				(*cloud) += *util3d::transformPointCloud(temp, pose);
#endif

				OdometryInfo info;
				mapBuilder.processOdometry(cameraData, pose, info);
			}

			cameraData = camera.takeImage();

			QApplication::processEvents();
			while(mapBuilder.isPaused() && mapBuilder.isVisible()) {
				uSleep(100);
				QApplication::processEvents();
			}
		}

		if(mapBuilder.isVisible()) {
			app.exec();
		}

	} else UERROR("Camera init failed!");

#ifdef SAVE_POINT_CLOUD
	util3d::voxelize(cloud, POINT_CLOUD_VOXEL_SIZE);
	pcl::io::savePCDFile(POINT_CLOUD_OUT_PATH, *cloud);
#endif

	return 0;
}