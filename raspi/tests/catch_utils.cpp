#include <catch_utils.hpp>

namespace catch_utils
{
std::string read_file(std::filesystem::path path)
{
    std::ifstream file(path);
    std::stringstream buf;

    if (file.fail())
    {
        throw std::runtime_error("File does not exist");
    }
    buf << file.rdbuf();

    return buf.str();
}

bool str_contains_all(std::string str, std::vector<std::string> &contains)
{
    return std::all_of(
        contains.cbegin(), contains.cend(), [str](const std::string &s) { return str.find(s) != std::string::npos; });
}

bool str_contains_non(std::string str, std::vector<std::string> &contains)
{
    return std::all_of(
        contains.cbegin(), contains.cend(), [str](const std::string &s) { return str.find(s) == std::string::npos; });
}

void set_env(std::string env, std::string val)
{
    setenv(env.c_str(), val.c_str(), 1);
}

TmpDir::TmpDir(bool remove_when_done) : _remove_when_done(remove_when_done)
{
    _path = TEST_ROOT / _folder_name();
    std::filesystem::create_directory(_path);
}

TmpDir::~TmpDir()
{
    if (_remove_when_done)
    {
        std::filesystem::remove_all(_path);
    }
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

PatchStdCout::PatchStdCout()
{
    _org_buf = std::cout.rdbuf();
    std::cout.rdbuf(_patch_buf.rdbuf());
}

PatchStdCout::~PatchStdCout()
{
    std::cout.rdbuf(_org_buf);
}

std::string PatchStdCout::get()
{
    return _patch_buf.str();
}
} // namespace catch_utils
