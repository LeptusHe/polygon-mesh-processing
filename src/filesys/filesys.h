#pragma once

#include <vector>
#include <string>

class FileManager  {
public:
    static void AddSearchPath(const std::string& search_path);
    static std::string FindFile(const std::string& file_name);

private:
    static std::string FindFileInDirectory(const std::string& file_name, const std::string& dir);

private:
    static std::vector<std::string> search_paths_;
};