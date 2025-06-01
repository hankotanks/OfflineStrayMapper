#ifndef IMU_THREAD_H
#define IMU_THREAD_H

#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UTimer.h>

#include <fstream>

class IMUThread : public UThread, public UEventsSender {
public:
    IMUThread(const double rate) : idx_(0), rate_(rate), previousStamp_(0.0) { /*  */ };
    ~IMUThread() = default;

    bool init(const std::vector<rtabmap::IMUEvent> imuData);

private:
    void mainLoopBegin();
    void mainLoop();

private:
    size_t idx_;
    double rate_;
    std::vector<rtabmap::IMUEvent> imuData_;
    UTimer frameRateTimer_;
    double previousStamp_;
};

#endif // IMU_THREAD_H