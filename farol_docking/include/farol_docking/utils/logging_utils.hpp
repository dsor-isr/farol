#pragma once

#include <iostream>

#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"

#define FAROL_ERROR(msg) \
  (std::cerr << COLOR_RED << "[ERROR] " << msg << COLOR_RESET << std::endl)

#define FAROL_WARN(msg) \
  (std::cerr << COLOR_YELLOW << "[WARN] " << msg << COLOR_RESET << std::endl)

#define FAROL_INFO(msg) \
  (std::cout << COLOR_GREEN << "[INFO] " << msg << COLOR_RESET << std::endl)
