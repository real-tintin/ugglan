#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <filesystem>
#include <vector>
#include <iostream>

namespace catch_utils
{
    inline const std::filesystem::path TEST_ROOT = "./tests";
    inline const std::filesystem::path RESOURCES_ROOT = "./tests/resources";

    std::string read_file(std::filesystem::path path);

    bool str_contains_all(std::string str, std::vector<std::string> contains);
    bool str_contains_non(std::string str, std::vector<std::string> contains);

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

    class PatchStdCout
    {
    public:
        PatchStdCout();
        ~PatchStdCout();

        std::string get();
    private:
        std::ostringstream _patch_buf;
        std::streambuf* _org_buf;
    };
}
