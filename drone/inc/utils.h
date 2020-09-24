#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <cstdlib>
#include "logger.h"

extern Logger logger;

std::string get_env_str(std::string env);

#endif /* UTILS_H */
