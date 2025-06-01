#include "IMUThread.h"
#include <rtabmap/core/IMUThread.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>

bool IMUThread::init(const std::vector<rtabmap::IMUEvent> imuData) {
    imuData_ = imuData;
    return true;
}

void IMUThread::mainLoopBegin() {
	ULogger::registerCurrentThread("IMU");
	frameRateTimer_.start();
}

void IMUThread::mainLoop() {
	UTimer totalTime;
	UDEBUG("");

	if(rate_ > 0.0 && false) {
		double delay = 1000.0 / rate_;
		int sleepTime = (int) (delay - 1000.0 * frameRateTimer_.getElapsedTime());
		if(sleepTime > 2) uSleep(sleepTime - 2);

		delay /= 1000.0;
		while(frameRateTimer_.getElapsedTime() < delay - 0.000001);

		frameRateTimer_.start();
	}

	if(idx_ < imuData_.size()) {
        this->post(&(imuData_[idx_++]));
    } else if(!this->isKilled()) {
		UWARN("no more imu data...");
		this->kill();
		this->post(new rtabmap::IMUEvent());
	}
}