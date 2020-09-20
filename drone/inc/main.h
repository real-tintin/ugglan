#ifndef MAIN_H
#define MAIN_H

#include <cstdint>
#include <string>
#include <cstdlib>
#include <iostream>
#include <filesystem>

inline const std::string ENV_DATA_LOG_ROOT = "DATA_LOG_ROOT";

std::string get_env_str(std::string env);

#endif /* MAIN_H */
