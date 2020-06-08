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

#include <fcntl.h>
#include <glob.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <serial/serial.h>

using namespace serial;

int PortInfo::getPortInfo(const std::string &serial_port_name) {
  std::smatch serial_port_match;
  std::regex_match(serial_port_name, serial_port_match, std::regex("^/dev/([^/]+)/?$"));
  if (serial_port_match.size() != 2) {
    //TODO: wrong serial port name
    return -1;
  }
  std::string serial_port_substr(serial_port_match[1]);

  std::string link_path;
  std::string system_path("/sys/class/tty/" + serial_port_substr);
  if (getLinkPath(system_path, link_path)) {
    //TODO: device not found in sys or link read error
    return -1;
  }

  if (std::strstr(link_path.c_str(), "usb")) {
    system_path = "/sys/class/tty/" + serial_port_substr + "/device";
    for (int i=0; i<3; i++) {
      system_path += "/..";
      //FIXME: should check the existence at least for the first four files
      struct stat serial_port_stat;
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
  serial_port = serial_port_name;
  return 0;
}

int serial::getLinkPath(std::string system_path, std::string &link_path) {
  struct stat serial_port_stat;
  if (::lstat(system_path.c_str(), &serial_port_stat) == -1) {
    return -1;
  }
  if (!S_ISLNK(serial_port_stat.st_mode)) {
    system_path += "/device";
  }
  char link_path_buf[PATH_MAX];
  ssize_t link_length = ::readlink(system_path.c_str(), link_path_buf, sizeof(link_path_buf) - 1);
  if (link_length == -1) {
    return -1;
  }
  link_path_buf[link_length] = '\0';
  link_path = std::string(link_path_buf);
  return 0;
}

int serial::getPortsInfo(std::vector<PortInfo> &serial_ports) {
  serial_ports.clear();
  std::vector<std::string> serial_port_names;
  if (getPortsList(serial_port_names) < 0) {
    return -1;
  }
  for (auto const &serial_port_name : serial_port_names) {
    PortInfo serial_port;
    if (!serial_port.getPortInfo(serial_port_name)) {
      serial_ports.push_back(serial_port);
    }
  }
  return serial_ports.size();
}

int serial::getPortsList(std::vector<std::string> &serial_port_names) {
  serial_port_names.clear();

  glob_t glob_results;
  std::string glob_pattern("/sys/class/tty/*");
  if (::glob(glob_pattern.c_str(), 0, nullptr, &glob_results)) {
    // 'serial_port_names' is cleared
    globfree(&glob_results);
    return -1;
  }

  for (int i=0; i<glob_results.gl_pathc; i++) {
    std::string link_path;
    std::string system_path(glob_results.gl_pathv[i]);
    if (getLinkPath(system_path, link_path)) {
      continue;
    }
    if (std::strstr(link_path.c_str(), "virtual")) {
      continue;
    }
    std::string serial_port_name("/dev/" + std::string(system_path.c_str() + std::strlen("/sys/class/tty/")));
    if (std::strstr(link_path.c_str(), "serial8250")) {
      int file_descriptor = ::open(serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (file_descriptor < 0) {
        continue;
      }
      struct serial_struct serial_info;
      if (::ioctl(file_descriptor, TIOCGSERIAL, &serial_info) || serial_info.type == PORT_UNKNOWN) {
        ::close(file_descriptor);
        continue;
      }
      ::close(file_descriptor);
    }
    serial_port_names.push_back(serial_port_name);
  }

  globfree(&glob_results);
  return serial_port_names.size();
}

#endif // defined(__linux__)
