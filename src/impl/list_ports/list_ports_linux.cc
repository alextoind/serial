/*
 * Copyright (c) 2014 Craig Lilley <cralilley@gmail.com>
 * This software is made available under the terms of the MIT licence.
 * A copy of the licence can be obtained from:
 * http://opensource.org/licenses/MIT
 */

#if defined(__linux__)

#include <climits>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include <glob.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <serial/serial.h>

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

int PortInfo::getPortInfo(const std::string &serial_port) {
  std::smatch serial_port_match;
  std::regex_match(serial_port, serial_port_match, std::regex("^/dev/([^/]+)/?$"));
  if (serial_port_match.size() != 2) {
    //TODO: wrong serial port name
    return -1;
  }
  std::string serial_port_name(serial_port_match[1]);

  struct stat serial_port_stat;
  std::string system_path("/sys/class/tty/" + serial_port_name);
  if (::lstat(system_path.c_str(), &serial_port_stat) == -1) {
    //TODO: device not found in sys
    return -1;
  }
  if (!S_ISLNK(serial_port_stat.st_mode)) {
    system_path = "/sys/class/tty/" + serial_port_name + "/device";
  }
  char link_path[PATH_MAX];
  ssize_t link_length = ::readlink(system_path.c_str(), link_path, sizeof(link_path) - 1);
  if (link_length == -1) {
    //TODO: link read error
    return -1;
  }
  link_path[link_length] = '\0';

  if (std::strstr(link_path, "usb")) {
    system_path = "/sys/class/tty/" + serial_port_name + "/device";
    for (int i=0; i<3; i++) {
      system_path += "/..";
      //FIXME: should check the existence at least for the first four files
      if (::stat((system_path + "/busnum").c_str(), &serial_port_stat) == -1) {
        continue;
      }
      std::ifstream(system_path + "/busnum") >> busnum;
      std::ifstream(system_path + "/devnum") >> devnum;
      std::ifstream(system_path + "/idProduct") >> id_product;
      std::ifstream(system_path + "/idVendor") >> id_vendor;
      std::getline(std::ifstream(system_path + "/manufacturer"), manufacturer);
      std::getline(std::ifstream(system_path + "/product"), product);
      std::getline(std::ifstream(system_path + "/serial"), serial_number);
      break;
    }
  }

  this->serial_port = serial_port;
  return 0;
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
    device_entry.getPortInfo(device);

    results.push_back(device_entry);
  }

  return results;
}

#endif // defined(__linux__)
