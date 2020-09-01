/***
 *  MIT License
 *
 *  Copyright (c) 2020 Alessandro Tondo
 *  Copyright (c) 2012 William Woodall, John Harrison
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 *  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 *  to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 *  the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 *  THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#if defined(__APPLE__)

#include <sys/param.h>
#include <stdint.h>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>

#include <serial/serial.h>

using namespace serial;

#define HARDWARE_ID_STRING_LENGTH 128

std::string cfstring_to_string(CFStringRef cfstring);
std::string get_device_path(io_object_t &serial_port);
std::string get_class_name(io_object_t &obj);
io_registry_entry_t get_parent_iousb_device(io_object_t &serial_port);
std::string get_string_property(io_object_t &device, const char *property);
uint16_t get_int_property(io_object_t &device, const char *property);
std::string rtrim(const std::string &str);

std::string cfstring_to_string(CFStringRef cfstring) {
  char cstring[MAXPATHLEN];
  std::string result;

  if (cfstring) {
    Boolean success = CFStringGetCString(cfstring, cstring, sizeof(cstring), kCFStringEncodingASCII);
    if (success) {
      result = cstring;
    }
  }

  return result;
}

std::string get_device_path(io_object_t &serial_port) {
  CFTypeRef callout_path;
  std::string device_path;
  callout_path = IORegistryEntryCreateCFProperty(serial_port, CFSTR(kIOCalloutDeviceKey), kCFAllocatorDefault, 0);

  if (callout_path) {
    if (CFGetTypeID(callout_path) == CFStringGetTypeID()) {
      device_path = cfstring_to_string(static_cast<CFStringRef>(callout_path));
    }

    CFRelease(callout_path);
  }

  return device_path;
}

std::string get_class_name(io_object_t &obj) {
  std::string result;
  io_name_t class_name;
  kern_return_t kern_result;
  kern_result = IOObjectGetClass(obj, class_name);

  if (kern_result == KERN_SUCCESS) {
    result = class_name;
  }

  return result;
}

io_registry_entry_t get_parent_iousb_device(io_object_t &serial_port) {
  io_object_t device = serial_port;
  io_registry_entry_t parent = 0;
  io_registry_entry_t result = 0;
  kern_return_t kern_result = KERN_FAILURE;
  std::string name = get_class_name(device);

  // Walk the IO Registry tree looking for this devices parent IOUSBDevice.
  while (name != "IOUSBDevice") {
    kern_result = IORegistryEntryGetParentEntry(device, kIOServicePlane, &parent);

    if (kern_result != KERN_SUCCESS) {
      result = 0;
      break;
    }

    device = parent;
    name = get_class_name(device);
  }

  if (kern_result == KERN_SUCCESS) {
    result = device;
  }

  return result;
}

std::string get_string_property(io_object_t &device, const char *property) {
  std::string property_name;

  if (device) {
    CFStringRef property_as_cfstring = CFStringCreateWithCString(kCFAllocatorDefault, property, kCFStringEncodingASCII);
    CFTypeRef name_as_cfstring = IORegistryEntryCreateCFProperty(device, property_as_cfstring, kCFAllocatorDefault, 0);

    if (name_as_cfstring) {
      if (CFGetTypeID(name_as_cfstring) == CFStringGetTypeID()) {
        property_name = cfstring_to_string(static_cast<CFStringRef>(name_as_cfstring));
      }
      CFRelease(name_as_cfstring);
    }

    if (property_as_cfstring) {
      CFRelease(property_as_cfstring);
    }
  }

  return property_name;
}

uint16_t get_int_property(io_object_t &device, const char *property) {
  uint16_t result = 0;

  if (device) {
    CFStringRef property_as_cfstring = CFStringCreateWithCString(kCFAllocatorDefault, property, kCFStringEncodingASCII);
    CFTypeRef number = IORegistryEntryCreateCFProperty(device, property_as_cfstring, kCFAllocatorDefault, 0);

    if (property_as_cfstring) {
      CFRelease(property_as_cfstring);
    }

    if (number) {
      if (CFGetTypeID(number) == CFNumberGetTypeID()) {
        bool success = CFNumberGetValue(static_cast<CFNumberRef>(number), kCFNumberSInt16Type, &result);
        if (!success) {
          result = 0;
        }
      }
      CFRelease(number);
    }
  }

  return result;
}

std::string rtrim(const std::string &str) {
  std::string result = str;
  std::string whitespace = " \t\f\v\n\r";
  std::size_t found = result.find_last_not_of(whitespace);

  if (found != std::string::npos) {
    result.erase(found + 1);
  } else {
    result.clear();
  }

  return result;
}

int PortInfo::getPortInfo(const std::string &serial_port_name) {
  //TODO
  return 0;
}

int serial::getPortsInfo(std::vector<PortInfo> &serial_ports) {
  serial_ports.clear();

  CFMutableDictionaryRef classes_to_match;
  io_iterator_t serial_port_iterator;
  io_object_t serial_port;
  mach_port_t master_port;
  kern_return_t kern_result;
  kern_result = IOMasterPort(MACH_PORT_NULL, &master_port);

  if (kern_result != KERN_SUCCESS) {
    return -1;
  }

  classes_to_match = IOServiceMatching(kIOSerialBSDServiceValue);
  if (classes_to_match == NULL) {
    return -1;
  }

  CFDictionarySetValue(classes_to_match, CFSTR(kIOSerialBSDTypeKey), CFSTR(kIOSerialBSDAllTypes));
  kern_result = IOServiceGetMatchingServices(master_port, classes_to_match, &serial_port_iterator);
  if (KERN_SUCCESS != kern_result) {
    return -1;
  }

  while ((serial_port = IOIteratorNext(serial_port_iterator))) {
    std::string device_path = get_device_path(serial_port);
    io_registry_entry_t parent = get_parent_iousb_device(serial_port);
    IOObjectRelease(serial_port);

    if (device_path.empty()) {
      continue;
    }

    PortInfo port_info;
    port_info.id_product = get_int_property(parent, "idProduct");
    port_info.id_vendor = get_int_property(parent, "idVendor");
    port_info.manufacturer = rtrim(get_string_property(parent, "USB Vendor Name"));
    port_info.product = rtrim(get_string_property(parent, "USB Product Name"));
    port_info.serial_number = rtrim(get_string_property(parent, "USB Serial Number"));
    port_info.serial_port = device_path;
    serial_ports.push_back(port_info);
  }

  IOObjectRelease(serial_port_iterator);
  return serial_ports.size();
}

int serial::getPortsList(std::vector<std::string> &serial_port_names) {
  serial_port_names.clear();
  //TODO
  return 0;
}

#endif // defined(__APPLE__)
