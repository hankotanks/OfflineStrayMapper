#ifndef STRAY_MAPPER_H
#define STRAY_MAPPER_H

#include <optional>
#include <string>

class StrayMapper {
private:
    std::string dataPath_;
    bool showUI_;
    bool savePointCloud_;
    bool keepTempFiles_;
    std::optional<std::string> outPath_;
    float voxelSize_;
public:
    StrayMapper(
        std::string dataPath,
        bool showUI, 
        bool savePointCloud,
        bool keepTempFiles,
        std::optional<std::string> outPath = {}
    ) : dataPath_(dataPath), showUI_(showUI), savePointCloud_(savePointCloud), keepTempFiles_(keepTempFiles), outPath_(outPath) { /*  */ }

    StrayMapper(int argc, char* argv[]);
    
    StrayMapper(const StrayMapper&) = delete;
    StrayMapper(StrayMapper&&) = delete;
    void operator=(const StrayMapper&) = delete;
    void operator=(StrayMapper&&) = delete;

    static void run(int argc, char* argv[]);
};

#endif // STRAY_MAPPER_H
