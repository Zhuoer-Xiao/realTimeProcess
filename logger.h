#pragma once
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <fstream>
#include <iostream>

class Logger {
public:
    Logger(const std::string& filePath = "myLog.log");
    ~Logger();

    void log(const std::string& message);

private:
    std::ofstream logFile;
};

#endif // LOGGER_H