#pragma once

#include <iostream>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

using Mesh = OpenMesh::TriMesh_ArrayKernelT<>;

constexpr const char* kLoggerName = "logger";

inline bool InitFileLogger(const std::string& file_path)
{
    try {
        auto logger = spdlog::basic_logger_mt(kLoggerName, file_path);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "failed to create logger" << std::endl;
        return false;
    }
    return true;
}

inline auto GetLogger() -> std::shared_ptr<spdlog::logger>
{
    return spdlog::get(kLoggerName);
}