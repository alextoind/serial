#if defined(__linux__)

/*
 * Copyright (c) 2014 Craig Lilley <cralilley@gmail.com>
 * This software is made available under the terms of the MIT licence.
 * A copy of the licence can be obtained from:
 * http://opensource.org/licenses/MIT
 */

#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

#include <glob.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "serial/serial.h"

using namespace serial;

std::vector<std::string> PortInfo::glob(const std::vector<std::string> &patterns) {
  std::vector<std::string> paths_found;

  if (patterns.size() == 0) {
    return paths_found;
  }

  glob_t glob_results;
  int glob_retval = ::glob(patterns[0].c_str(), 0, NULL, &glob_results);

  std::vector<std::string>::const_iterator iter = patterns.begin();
  while (++iter != patterns.end()) {
    glob_retval = ::glob(iter->c_str(), GLOB_APPEND, NULL, &glob_results);
  }

  for (int path_index = 0; path_index < glob_results.gl_pathc; path_index++) {
    paths_found.push_back(glob_results.gl_pathv[path_index]);
  }

  globfree(&glob_results);
  return paths_found;
}

std::string PortInfo::basename(const std::string &path) {
  size_t pos = path.rfind("/");

  if (pos == std::string::npos) {
    return path;
  }

  return std::string(path, pos + 1, std::string::npos);
}

std::string PortInfo::dirname(const std::string &path) {
  size_t pos = path.rfind("/");

  if (pos == std::string::npos) {
    return path;
  } else if (pos == 0) {
    return "/";
  }

  return std::string(path, 0, pos);
}

bool PortInfo::path_exists(const std::string &path) {
  struct stat sb;

  if (stat(path.c_str(), &sb) == 0) {
    return true;
  }

  return false;
}

std::string PortInfo::realpath(const std::string &path) {
  char *real_path = ::realpath(path.c_str(), NULL);
  std::string result;

  if (real_path != NULL) {
    result = real_path;
    free(real_path);
  }

  return result;
}

std::vector<std::string> PortInfo::get_sysfs_info(const std::string &device_path) {
  std::string device_name = basename(device_path);
  std::string friendly_name;
  std::string hardware_id;
  std::string sys_device_path = format("/sys/class/tty/%s/device", device_name.c_str());

  if (device_name.compare(0, 6, "ttyUSB") == 0) {
    sys_device_path = dirname(dirname(realpath(sys_device_path)));

    if (path_exists(sys_device_path)) {
      getPortInfo(sys_device_path);
    }
  } else if (device_name.compare(0, 6, "ttyACM") == 0) {
    sys_device_path = dirname(realpath(sys_device_path));

    if (path_exists(sys_device_path)) {
      getPortInfo(sys_device_path);
    }
  } else {
    // Try to read ID std::string of PCI device
    std::string sys_id_path = sys_device_path + "/id";

    if (path_exists(sys_id_path)) {
      std::ifstream(sys_id_path, std::ifstream::in) >> product;
    }
  }

  if (friendly_name.empty()) {
    friendly_name = device_name;
  }

  if (hardware_id.empty()) {
    hardware_id = "n/a";
  }

  std::vector<std::string> result;
  result.push_back(friendly_name);
  result.push_back(hardware_id);

  return result;
}

std::string PortInfo::format(const char *format, ...) {
  va_list ap;
  size_t buffer_size_bytes = 256;
  std::string result;
  char *buffer = (char *)malloc(buffer_size_bytes);

  if (buffer == NULL) {
    return result;
  }

  bool done = false;
  unsigned int loop_count = 0;
  while (!done) {
    va_start(ap, format);
    int return_value = vsnprintf(buffer, buffer_size_bytes, format, ap);

    if (return_value < 0) {
      done = true;
    } else if (return_value >= buffer_size_bytes) {
      // Realloc and try again.
      buffer_size_bytes = return_value + 1;
      char *new_buffer_ptr = (char *)realloc(buffer, buffer_size_bytes);

      if (new_buffer_ptr == NULL) {
        done = true;
      } else {
        buffer = new_buffer_ptr;
      }
    } else {
      result = buffer;
      done = true;
    }

    va_end(ap);

    if (++loop_count > 5) {
      done = true;
    }
  }

  free(buffer);
  return result;
}

void PortInfo::getPortInfo(const std::string &sysfs_path) {
  std::ifstream(sysfs_path + "/busnum", std::ifstream::in) >> busnum;
  std::ifstream(sysfs_path + "/devnum", std::ifstream::in) >> devnum;
  std::ifstream(sysfs_path + "/idProduct", std::ifstream::in) >> id_product;
  std::ifstream(sysfs_path + "/idVendor", std::ifstream::in) >> id_vendor;
  std::ifstream(sysfs_path + "/manufacturer", std::ifstream::in) >> manufacturer;
  std::ifstream(sysfs_path + "/product", std::ifstream::in) >> product;
  std::ifstream(sysfs_path + "/serial", std::ifstream::in) >> serial_number;
}

std::vector<PortInfo> serial::list_ports() {
  std::vector<PortInfo> results;

  std::vector<std::string> search_globs;
  search_globs.push_back("/dev/ttyACM*");
  search_globs.push_back("/dev/ttyS*");
  search_globs.push_back("/dev/ttyUSB*");
  search_globs.push_back("/dev/tty.*");
  search_globs.push_back("/dev/cu.*");

  std::vector<std::string> devices_found = PortInfo::glob(search_globs);  //FIXME: maybe in PortInfo constructor
  std::vector<std::string>::iterator iter = devices_found.begin();

  while (iter != devices_found.end()) {
    std::string device = *iter++;

    PortInfo device_entry;
    device_entry.serial_port = device;
    device_entry.get_sysfs_info(device);

    results.push_back(device_entry);
  }

  return results;
}

#endif // defined(__linux__)
