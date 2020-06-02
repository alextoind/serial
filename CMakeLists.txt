cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(Serial VERSION 2.0.0 LANGUAGES CXX)

if(APPLE)
  #TODO: check if both are required and if find_package instead of find_library works
  find_package(IOKit REQUIRED)
  find_package(Foundation REQUIRED)
elseif(UNIX)
#  find_package(rt REQUIRED)  #FIXME: not as package or module
#  find_package(Threads REQUIRED)  #TODO: seems no longer required
elseif(WIN32)
  #TODO: check if it is required and if find_package works
  find_package(setupapi REQUIRED)
else()
  message(FATAL_ERROR "Not supported platform.")
endif()

add_library(Serial SHARED
  src/serial.cc
)
target_include_directories(Serial PUBLIC
  $<BUILD_INTERFACE:${Serial_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${Serial_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(APPLE)
  target_sources(Serial PRIVATE
    src/impl/impl_unix.cc
    src/impl/list_ports/list_ports_osx.cc
  )
  target_link_libraries(Serial PRIVATE
    IOKit
    Foundation
  )
elseif(UNIX)
  target_sources(Serial PRIVATE
    src/impl/impl_unix.cc
    src/impl/list_ports/list_ports_linux.cc
  )
#TODO: seems no longer required
#  target_link_libraries(Serial PRIVATE
#    rt
#    Threads::Threads
#  )
else()  # already asserted WIN32 during find_package block
  target_sources(Serial PRIVATE
    src/impl/impl_win.cc
    src/impl/list_ports/list_ports_win.cc
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

install(TARGETS Serial EXPORT SerialTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
find_package(GTest REQUIRED)

add_executable(unix_serial_tests
  tests/unix_serial_tests.cc
)
target_link_libraries(unix_serial_tests PRIVATE
  Serial
  GTest::GTest
  GTest::Main
  util  #FIXME: this is just to use the already existing tests
)
add_test(NAME unix_serial_tests
  COMMAND unix_serial_tests
)

add_executable(unix_timer_tests
  tests/unix_timer_tests.cc
)
target_link_libraries(unix_timer_tests PRIVATE
  Serial
  GTest::GTest
  GTest::Main
)
add_test(NAME unix_timer_tests
  COMMAND unix_timer_tests
)