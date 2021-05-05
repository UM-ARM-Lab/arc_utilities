#include <arc_utilities/text_color.hpp>
#include <iostream>

int main() {
  using namespace arc_color;
  std::cout << "This text should be normal\n";
  std::cout << RED << "This text should be red" << RESET << "\n";
  std::cout << "This text should be normal again"
            << "\n";
  std::cout << "The following text should be " << GREEN << "green, " << BLUE << "blue, " << CYAN << "cyan, " << MAGENTA
            << "magenta, " << YELLOW << "yellow, " << WHITE << "white" << RESET << "\n";
  std::cout << "This following should be " << UNDERLINED << "underlined" << RESET << ", " << UNDERLINED << RED
            << "underlined and red" << RESET << ", " << BOLD << "bold" << RESET << ", " << BOLD << UNDERLINED << RED
            << "bold, underlined, and red" << RESET << "\n";
  std::cout << "And everything should be back to normal\n";
}
