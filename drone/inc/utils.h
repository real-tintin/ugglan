#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <cstdlib>
#include "spdlog/spdlog.h"

inline const auto logger = spdlog::default_logger_raw();

std::string get_env_str(std::string env);

#endif /* UTILS_H */
