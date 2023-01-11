#ifndef MONO_SLAM_UTILS_H
#define MONO_SLAM_UTILS_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <type_traits>

namespace utils
{
    using Timestamp = std::uint64_t;
    using Index     = std::uint32_t;

    enum class BackendState
    {
        INITIALIZING = 0,
        RUNNING      = 1,
    };
}

#endif