#ifndef PY_SCRIPT_H
#define PY_SCRIPT_H

#include <string>
#undef slots
#include <Python.h>

class PyScript {
private:
    std::string moduleName_;
    PyObject* module_;
    bool errOnLastCall_ = false;
public:
    PyScript(const char* moduleName);
    ~PyScript();
    unsigned int call(const char* name, const char* format, ...);
    void printErr();
};

#endif // PY_SCRIPT_H