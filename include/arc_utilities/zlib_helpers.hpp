#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>

#ifndef ZLIB_HELPERS_HPP
#define ZLIB_HELPERS_HPP

namespace ZlibHelpers
{
    std::vector<u_int8_t> DecompressBytes(const std::vector<u_int8_t>& compressed);

    std::vector<u_int8_t> CompressBytes(const std::vector<u_int8_t>& uncompressed);
}

#endif // ZLIB_HELPERS_HPP
