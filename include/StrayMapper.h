#ifndef STRAY_MAPPER_H
#define STRAY_MAPPER_H

#include <optional>
#include <string>

class StrayMapper {
private:
    std::string appName_;
    std::string dataPath_;
    bool showInfo_ = false;
    bool showUI_ = false;
    bool savePCD_ = false;
    bool saveVTK_ = false;
    bool keepTempFiles_ = false;
    bool upscaleDepth_ = false;
    bool upscaleNoPromptDA_ = true;
    std::optional<std::string> outPath_ = {};
public:
    StrayMapper(int argc, char* argv[]);
    // remove all autodef constructors
    StrayMapper() = delete;
    StrayMapper(const StrayMapper&) = delete;
    StrayMapper(StrayMapper&&) = delete;
    void operator=(const StrayMapper&) = delete;
    void operator=(StrayMapper&&) = delete;
    // run the SLAM process
    unsigned int run() const;
private:
    // internal constructor (after cli-arg processing)
    StrayMapper(
        std::string dataPath,
        bool showUI, 
        bool savePCD,
        bool saveVTK,
        bool keepTempFiles,
        std::optional<std::string> outPath = {}
    ) : dataPath_(dataPath), 
        showUI_(showUI), 
        savePCD_(savePCD), saveVTK_(saveVTK), 
        keepTempFiles_(keepTempFiles), 
        outPath_(outPath) { /*  */ }
    // validate the StrayMapper instance before running
    unsigned int validate() const;
};

#endif // STRAY_MAPPER_H
