#include "Logger.h"
#include <chrono>
#include <iomanip>
#include <sstream>

Logger::Logger(const std::string& filePath) {
    logFile.open(filePath, std::ios::out | std::ios::app);
    if (!logFile.is_open()) {
        throw std::runtime_error("Could not open the log file.");
    }
}

Logger::~Logger() {
    if (logFile.is_open()) {
        logFile.close();
    }
}

void Logger::log(const std::string& message) {
    if (logFile.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

        std::stringstream timeStream;
        timeStream << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
        timeStream << '.' << std::setfill('0') << std::setw(3) << millis;

        logFile << timeStream.str() << " - " << message << std::endl;
        logFile.flush(); // 确保数据被立即写入文件
    }
}