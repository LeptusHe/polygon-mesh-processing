#include "filesys.h"
#include <filesystem>

namespace fs = std::filesystem;

std::vector<std::string> FileManager::search_paths_;


void FileManager::AddSearchPath(const std::string& search_path)
{
    search_paths_.push_back(search_path);
}

std::string FileManager::FindFile(const std::string& file_name)
{
    if (fs::exists(file_name)) {
        return file_name;
    }

    for (const auto& search_path : search_paths_) {
        auto path = FindFileInDirectory(file_name, search_path);
        if (!path.empty()) {
            return fs::absolute(path).string();
        }
    }
    return "";
}

std::string FileManager::FindFileInDirectory(const std::string& file_name, const std::string& dir)
{
    auto directory = fs::path(dir);
    for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
        if (entry.is_directory()) {
            auto abs_file_path = FindFileInDirectory(file_name, fs::absolute(entry.path()).string());
            if (!abs_file_path.empty()) {
                return abs_file_path;
            }
        } else {
            if (entry.path().filename() == file_name) {
                return fs::absolute(entry.path()).string();
            }
        }
    }
    return "";
}