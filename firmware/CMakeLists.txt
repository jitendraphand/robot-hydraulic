cmake_minimum_required(VERSION 3.16)
project(hx_firmware_host CXX)

# Host build of the portable firmware core:
#   hxcore      static library (same sources as the ESP32 build)
#   hx_sil      shared library with a C API for the Python SIL harness
#   hx_tests    unit tests (ctest)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(WARN -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion -Werror -Wno-error=conversion)

add_library(hxcore STATIC core/src/control.cpp)
target_include_directories(hxcore PUBLIC core/include)
target_compile_options(hxcore PRIVATE ${WARN} -fno-exceptions)

add_library(hx_sil SHARED sil/hx_sil.cpp)
target_link_libraries(hx_sil PRIVATE hxcore)

add_executable(hx_tests tests/test_main.cpp tests/test_control.cpp tests/test_config.cpp)
target_link_libraries(hx_tests PRIVATE hxcore)
target_compile_options(hx_tests PRIVATE -Wall -Wextra)

enable_testing()
add_test(NAME hx_unit COMMAND hx_tests)
