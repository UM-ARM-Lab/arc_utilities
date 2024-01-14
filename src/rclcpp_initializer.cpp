#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace py = pybind11;

void init_rclcpp(std::string const &name, std::vector<std::string> argv = {}) {
  auto argc = static_cast<int>(argv.size());
  std::vector<char *> argv_pointers;
  auto const convert = [](const std::string &s) {
    char *pc = new char[s.size() + 1];
    std::strcpy(pc, s.c_str());
    return pc;
  };
  std::transform(argv.begin(), argv.end(), std::back_inserter(argv_pointers), convert);
  rclcpp::init(argc, argv_pointers.data());
}

void shutdown() { rclcpp::shutdown(); }

PYBIND11_MODULE(roscpp_initializer, m) {

  m.doc() = "rclcpp initializer module";

  m.def("init", &init_rclcpp, "init rclcpp for C++", py::arg("name"), py::arg("argv") = std::vector<std::string>{});
  m.def("shutdown", &shutdown, "shutdown");
}
