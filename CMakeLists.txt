cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(Serial VERSION 2.0.0 LANGUAGES CXX)

if(APPLE)
  #TODO: check if both are required and if find_package instead of find_library works
  find_package(IOKit REQUIRED)
  find_package(Foundation REQUIRED)
elseif(UNIX)
  #find_package(rt REQUIRED)  #FIXME: not as package or moudle
  find_package(Threads REQUIRED)
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