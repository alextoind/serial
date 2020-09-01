###
#  MIT License
#
#  Copyright (c) 2020 Alessandro Tondo
#  Copyright (c) 2012 William Woodall, John Harrison
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
#  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
#  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
#  to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all copies or substantial portions of
#  the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
#  THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
#  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
#

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(Serial VERSION 2.0.0 LANGUAGES CXX)

if(APPLE)
  find_library(COREFOUNDATION_LIBRARY CoreFoundation)
  if (NOT COREFOUNDATION_LIBRARY)
    message(FATAL_ERROR "CoreFoundation framework not found.")
  endif()
  find_library(IOKIT_LIBRARY IOKit)
  if (NOT IOKIT_LIBRARY)
    message(FATAL_ERROR "IOKit framework not found.")
  endif()
elseif(UNIX)
  # noting to fetch
elseif(WIN32)
  if (MSVC_VERSION GREATER_EQUAL "1900")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
    if (_cpp_latest_flag_supported)
      add_compile_options("/std:c++latest")
    endif()
  endif()
else()
  message(FATAL_ERROR "Not supported platform.")
endif()

add_library(Serial SHARED
  src/serial.cpp
)
target_include_directories(Serial PUBLIC
  $<BUILD_INTERFACE:${Serial_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${Serial_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(APPLE)
  target_sources(Serial PRIVATE
    src/impl/impl_unix.cpp
    src/impl/list_ports/list_ports_macos.cpp
  )
  target_link_libraries(Serial PRIVATE
    "-framework CoreFoundation"
    "-framework IOKit"
  )
elseif(UNIX)
  target_sources(Serial PRIVATE
    src/impl/impl_unix.cpp
    src/impl/list_ports/list_ports_linux.cpp
  )
else()  # already asserted WIN32 during find_package block
  target_sources(Serial PRIVATE
    src/impl/impl_win.cpp
    src/impl/list_ports/list_ports_win.cpp
  )
  target_link_libraries(Serial PRIVATE
    setupapi
  )
endif()
# target_compile_features(Serial PUBLIC cxx_std_11) is available only from CMake 3.8
set_target_properties(Serial PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS NO
)
add_library(Serial::Serial ALIAS Serial)

install(DIRECTORY include/serial
  DESTINATION include
)
install(TARGETS Serial EXPORT SerialTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT SerialTargets
  FILE SerialTargets.cmake
  NAMESPACE Serial::
  DESTINATION lib/cmake/Serial
)

include(CMakePackageConfigHelpers)
configure_package_config_file(SerialConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SerialConfig.cmake
  INSTALL_DESTINATION lib/cmake/Serial
)
write_basic_package_version_file(SerialConfigVersion.cmake
  VERSION ${Serial_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/SerialConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/SerialConfigVersion.cmake
  DESTINATION lib/cmake/Serial
)

# Tests
enable_testing()
find_package(GTest REQUIRED CONFIG)

add_executable(unix_serial_tests
  tests/unix_serial_tests.cpp
)
target_link_libraries(unix_serial_tests PRIVATE
  Serial::Serial
  GTest::gtest
  GTest::gmock
  GTest::gtest_main
)
if(UNIX)
target_link_libraries(unix_serial_tests PRIVATE
  util  #FIXME: this is just to use the already existing tests
)
endif()
set_target_properties(unix_serial_tests PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS NO
)
add_test(NAME unix_serial_tests
  COMMAND unix_serial_tests
)

add_executable(unix_timer_tests
  tests/unix_timer_tests.cpp
)
target_link_libraries(unix_timer_tests PRIVATE
  Serial::Serial
  GTest::gtest
  GTest::gtest_main
)
set_target_properties(unix_timer_tests PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS NO
)
add_test(NAME unix_timer_tests
  COMMAND unix_timer_tests
)

add_executable(unix_list_ports_tests
  tests/unix_list_ports_tests.cpp
)
target_link_libraries(unix_list_ports_tests PRIVATE
  Serial::Serial
  GTest::gtest
  GTest::gmock
  GTest::gtest_main
)
set_target_properties(unix_list_ports_tests PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS NO
)
add_test(NAME unix_list_ports_tests
  COMMAND unix_list_ports_tests
)
