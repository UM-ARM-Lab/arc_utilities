#ifndef ARC_UTILITIES_TEXT_COLOR_HPP
#define ARC_UTILITIES_TEXT_COLOR_HPP

#include <string>

namespace arc_color {

constexpr auto RESET = "\x1B[0m";
constexpr auto RED = "\x1B[31m";
constexpr auto GREEN = "\x1B[32m";
constexpr auto YELLOW = "\x1B[33m";
constexpr auto BLUE = "\x1B[34m";
constexpr auto MAGENTA = "\x1B[35m";
constexpr auto CYAN = "\x1B[36m";
constexpr auto WHITE = "\x1B[37m";

constexpr auto BOLD = "\x1B[1m";
constexpr auto UNDERLINED = "\x1B[4m";

}  // namespace arc_color

#endif  // ARC_UTILITIES_TEXT_COLOR_HPP
