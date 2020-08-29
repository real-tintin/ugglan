#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <filesystem>

namespace catch_utils
{
    inline const std::filesystem::path TEST_ROOT = "./tests";
    inline const std::filesystem::path RESOURCES_ROOT = "./tests/resources";

    std::string read_file(std::filesystem::path path);

    class TmpDir
    {
    public:
        TmpDir(bool remove_when_done = true);
        ~TmpDir();

        std::filesystem::path get_path();
    private:
        std::filesystem::path _path;
        bool _remove_when_done;

        std::string _folder_name();
    };
}
