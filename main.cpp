
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/utilite/UThread.h>
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
	
	StrayCamera camera(argv[1]);

	if(camera.init()) {
		Rtabmap rtabmap;
		rtabmap.init();

		QApplication app(argc, argv);
		MapBuilder mapBuilder;
		mapBuilder.show();
		QApplication::processEvents();

		SensorData data = camera.takeImage();
		int cameraIteration = 0;
		int odometryIteration = 0;
		while(data.isValid() && mapBuilder.isVisible()) {
			if(++cameraIteration < camera.getFrameCount()) {
				Transform pose = camera.getPose(data.id());
				if(odometryIteration++) {
					if(rtabmap.process(data, pose)) {
						mapBuilder.processStatistics(rtabmap.getStatistics());
						if(rtabmap.getLoopClosureId() > 0) {
							printf("Loop closure detected!\n");
						}
					}
				}

				OdometryInfo info;
				mapBuilder.processOdometry(data, pose, info);
			}

			QApplication::processEvents();
			while(mapBuilder.isPaused() && mapBuilder.isVisible()) {
				uSleep(100);
				QApplication::processEvents();
			}
			data = camera.takeImage();
		}

		if(mapBuilder.isVisible()) {
			printf("Processed all frames\n");
			app.exec();
		}
	} else UERROR("Camera init failed!");

	return 0;
}