#include "util/PyScript.h"
#include <filesystem>
#include <string>
#include <cstdarg>

PyScript::PyScript(const char* moduleName) {
    std::string pathEnv{PY_PATH_ENV};
    std::string pathLib32(pathEnv), pathLib64(pathEnv);
    pathLib32 += "/lib";
    pathLib64 += "/lib64";
    std::string pyFolder = "";
    for(const auto& entry : std::filesystem::directory_iterator(pathLib32)) {
        if(entry.is_directory()) {
            std::string dir = entry.path().filename().string();
            if(dir.find("python") != std::string::npos) {
                pyFolder += dir;
                break;
            }
        }
    }

    if(pyFolder.empty()) {
        module_ = nullptr;
        return;
    }

    moduleName_ = std::string(moduleName);

    pathLib32 += "/" + pyFolder + "/site-packages";
    pathLib64 += "/" + pyFolder + "/site-packages";

    // Py_SetPythonHome(std::wstring(pathEnv.begin(), pathEnv.end()).c_str());

    PyConfig config;
    PyConfig_InitIsolatedConfig(&config);
    PyConfig_SetBytesString(&config, &config.program_name, moduleName);

    Py_InitializeFromConfig(&config);
    PyConfig_Clear(&config);

    const char* pathExt[] = {
        PY_PATH_SCRIPTS,
        pathLib32.c_str(),
        pathLib64.c_str(),
        PY_PATH_PROMPTDA_MODULE
    };

    PyObject* pyPathSys = PySys_GetObject("path");
    for(const char* path : pathExt) {
        PyObject* pyPath = PyUnicode_FromString(path);
        PyList_Append(pyPathSys, pyPath);
        Py_DECREF(pyPath);
    }

    PyObject* pyName = PyUnicode_DecodeFSDefault(moduleName);
    module_ = PyImport_Import(pyName);
    Py_DECREF(pyName);
}

PyScript::~PyScript() {
    Py_DECREF(module_);
    Py_Finalize();
}

unsigned int PyScript::call(const char* name, const char* format, ...) {
    errOnLastCall_ = false;

    if(module_ != NULL) {
        va_list args;
        int argc = (int) strlen(format);

        va_start(args, argc);
        PyObject* pyResult = PyObject_CallMethod(module_, name, format, args);
        va_end(args);

        if(pyResult == NULL) {
            errOnLastCall_ = true;
            return 1;
        } else Py_DECREF(pyResult);

    } else {
        errOnLastCall_ = true;
        return 1;
    }

    return 0;
}

void PyScript::printErr() {
    PyErr_Print();
}