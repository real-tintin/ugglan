#include <catch_utils.h>

namespace catch_utils
{
    std::string read_file(std::filesystem::path path)
    {
        std::ifstream file(path);
        std::stringstream buf;

        if (file.fail()) { throw std::runtime_error("File does not exist"); }
        buf << file.rdbuf();

        return buf.str();
    }

    TmpDir::TmpDir(bool remove_when_done) :
        _remove_when_done(remove_when_done)
    {
        _path = TEST_ROOT / _folder_name();
        std::filesystem::create_directory(_path);
    }

    TmpDir::~TmpDir()
    {
        if (_remove_when_done) { std::filesystem::remove_all(_path); }
    }

    std::filesystem::path TmpDir::get_path()
    {
        return _path;
    }

    std::string TmpDir::_folder_name()
    {
        std::stringstream buf;
        std::time_t now_time = std::time(nullptr);

        std::tm tm = *std::localtime(&now_time);
        buf << std::put_time(&tm, "%Y%m%d%H%M%S");

        return "tmpdir_" + buf.str();
    }
}
