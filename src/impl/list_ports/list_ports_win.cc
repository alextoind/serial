/***
 *  MIT License
 *
 *  Copyright (c) 2020 Alessandro Tondo
 *  Copyright (c) 2014 Craig Lilley
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

#if defined(_WIN32)

#include <tchar.h>
#include <windows.h>
#include <setupapi.h>
#include <initguid.h>
#include <devguid.h>
#include <cstring>

#include <serial/serial.h>

using namespace serial;

static const DWORD port_name_max_length = 256;
static const DWORD friendly_name_max_length = 256;
static const DWORD hardware_id_max_length = 256;

// Convert a wide Unicode string to an UTF8 string
std::string utf8_encode(const std::wstring &wstr) {
  int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
  std::string strTo(size_needed, 0);
  WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &strTo[0], size_needed, NULL, NULL);
  return strTo;
}

int serial::getPortsInfo(std::vector<PortInfo> &serial_ports) {
  serial_ports.clear();

  HDEVINFO device_info_set = SetupDiGetClassDevs((const GUID *)&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);
  unsigned int device_info_set_index = 0;
  SP_DEVINFO_DATA device_info_data;
  device_info_data.cbSize = sizeof(SP_DEVINFO_DATA);

  while (SetupDiEnumDeviceInfo(device_info_set, device_info_set_index, &device_info_data)) {
    device_info_set_index++;

    // Get port name
    HKEY hkey = SetupDiOpenDevRegKey(device_info_set, &device_info_data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
    TCHAR port_name[port_name_max_length];
    DWORD port_name_length = port_name_max_length;
    LONG return_code = RegQueryValueEx(hkey, _T("PortName"), NULL, NULL, (LPBYTE)port_name, &port_name_length);
    RegCloseKey(hkey);

    if (return_code != EXIT_SUCCESS) {
      continue;
    }

    if (port_name_length > 0 && port_name_length <= port_name_max_length) {
      port_name[port_name_length - 1] = '\0';
    } else {
      port_name[0] = '\0';
    }

    // Ignore parallel ports
    if (_tcsstr(port_name, _T("LPT")) != NULL) {
      continue;
    }

    // Get port friendly name
    TCHAR friendly_name[friendly_name_max_length];
    DWORD friendly_name_actual_length = 0;
    BOOL got_friendly_name = SetupDiGetDeviceRegistryProperty(device_info_set, &device_info_data, SPDRP_FRIENDLYNAME,
                                                              NULL, (PBYTE)friendly_name, friendly_name_max_length,
                                                              &friendly_name_actual_length);

    if (got_friendly_name == TRUE && friendly_name_actual_length > 0) {
      friendly_name[friendly_name_actual_length - 1] = '\0';
    } else {
      friendly_name[0] = '\0';
    }

    // Get hardware ID
    TCHAR hardware_id[hardware_id_max_length];
    DWORD hardware_id_actual_length = 0;
    BOOL got_hardware_id = SetupDiGetDeviceRegistryProperty(device_info_set, &device_info_data, SPDRP_HARDWAREID, NULL,
                                                            (PBYTE)hardware_id, hardware_id_max_length,
                                                            &hardware_id_actual_length);

    if (got_hardware_id == TRUE && hardware_id_actual_length > 0) {
      hardware_id[hardware_id_actual_length - 1] = '\0';
    } else {
      hardware_id[0] = '\0';
    }

#ifdef UNICODE
    std::string portName = utf8_encode(port_name);
    std::string friendlyName = utf8_encode(friendly_name);
    std::string hardwareId = utf8_encode(hardware_id);
#else
    std::string portName = port_name;
    std::string friendlyName = friendly_name;
    std::string hardwareId = hardware_id;
#endif

    PortInfo port_info;
    //FIXME
    //port_info.id_product = get_int_property(parent, "idProduct");
    //port_info.id_vendor = get_int_property(parent, "idVendor");
    //port_info.manufacturer = rtrim(get_string_property(parent, "USB Vendor Name"));
    //port_info.product = rtrim(get_string_property(parent, "USB Product Name"));
    //port_info.serial_number = rtrim(get_string_property(parent, "USB Serial Number"));
    port_info.serial_port = portName;
    serial_ports.push_back(port_info);
  }

  SetupDiDestroyDeviceInfoList(device_info_set);
  return serial_ports.size();
}

int serial::getPortsList(std::vector<std::string> &serial_port_names) {
  serial_port_names.clear();

  return 0;
}

#endif // #if defined(_WIN32)
