#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>
#include "arc_utilities/base64_helpers.hpp"

namespace Base64Helpers
{
    std::vector<uint8_t> Decode(const std::string& encoded)
    {
        (void)(encoded);
        return std::vector<uint8_t>();
    }

    std::string Encode(const std::vector<uint8_t>& binary)
    {
        (void)(binary);
        return std::string();
    }
}
