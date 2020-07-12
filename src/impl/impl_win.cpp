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

#if defined(_WIN32)

#include <serial/impl/impl.h>

using namespace serial;

inline std::wstring _prefix_port_if_needed(const std::wstring &input) {
  static std::wstring windows_com_port_prefix = L"\\\\.\\";
  if (input.compare(windows_com_port_prefix) != 0) {
    return windows_com_port_prefix + input;
  }
  return input;
}

Serial::SerialImpl::SerialImpl(const std::string &port, unsigned long baudrate, bytesize_t bytesize, parity_t parity,
                               stopbits_t stopbits, flowcontrol_t flowcontrol)
    : port_(port.begin(), port.end()),
      fd_(INVALID_HANDLE_VALUE),
      is_open_(false),
      baudrate_(baudrate),
      parity_(parity),
      bytesize_(bytesize),
      stopbits_(stopbits),
      flowcontrol_(flowcontrol) {
  if (!port_.empty()) {
    open();
  }
}

Serial::SerialImpl::~SerialImpl() {
  close();
}

void Serial::SerialImpl::open() {
  if (port_.empty()) {
    throw SerialInvalidArgumentException("empty port is invalid.");
  }
  if (is_open_) {
    throw SerialException("serial port already open.");  //TODO: actually there is no need to throw exception in this case
  }

  // See: https://github.com/wjwwood/serial/issues/84
  std::wstring port_with_prefix = _prefix_port_if_needed(port_);
  LPCWSTR lp_port = port_with_prefix.c_str();
  fd_ = CreateFileW(lp_port, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);

  if (fd_ == INVALID_HANDLE_VALUE) {
    if (GetLastError() == ERROR_FILE_NOT_FOUND) {
      throw SerialIOException("the specified port '" + getPort() + "' does not exist");
    } else{
      throw SerialIOException("error '" + std::to_string(GetLastError()) + "' while opening the serial port");
    }
  }

  reconfigurePort();
  is_open_ = true;
}

void Serial::SerialImpl::reconfigurePort() {
  if (fd_ == INVALID_HANDLE_VALUE) {
    throw SerialIOException("invalid file descriptor, is the serial port open?");
  }

  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (!GetCommState(fd_, &dcbSerialParams)) {
    throw SerialIOException("error getting the serial port state.");
  }

  // setup baud rate
  switch (baudrate_) {
#ifdef CBR_0
    case 0:
      dcbSerialParams.BaudRate = CBR_0;
      break;
#endif
#ifdef CBR_50
    case 50:
      dcbSerialParams.BaudRate = CBR_50;
      break;
#endif
#ifdef CBR_75
    case 75:
      dcbSerialParams.BaudRate = CBR_75;
      break;
#endif
#ifdef CBR_110
    case 110:
      dcbSerialParams.BaudRate = CBR_110;
      break;
#endif
#ifdef CBR_134
    case 134:
      dcbSerialParams.BaudRate = CBR_134;
      break;
#endif
#ifdef CBR_150
    case 150:
      dcbSerialParams.BaudRate = CBR_150;
      break;
#endif
#ifdef CBR_200
    case 200:
      dcbSerialParams.BaudRate = CBR_200;
      break;
#endif
#ifdef CBR_300
    case 300:
      dcbSerialParams.BaudRate = CBR_300;
      break;
#endif
#ifdef CBR_600
    case 600:
      dcbSerialParams.BaudRate = CBR_600;
      break;
#endif
#ifdef CBR_1200
    case 1200:
      dcbSerialParams.BaudRate = CBR_1200;
      break;
#endif
#ifdef CBR_1800
    case 1800:
      dcbSerialParams.BaudRate = CBR_1800;
      break;
#endif
#ifdef CBR_2400
    case 2400:
      dcbSerialParams.BaudRate = CBR_2400;
      break;
#endif
#ifdef CBR_4800
    case 4800:
      dcbSerialParams.BaudRate = CBR_4800;
      break;
#endif
#ifdef CBR_7200
    case 7200:
      dcbSerialParams.BaudRate = CBR_7200;
      break;
#endif
#ifdef CBR_9600
    case 9600:
      dcbSerialParams.BaudRate = CBR_9600;
      break;
#endif
#ifdef CBR_14400
    case 14400:
      dcbSerialParams.BaudRate = CBR_14400;
      break;
#endif
#ifdef CBR_19200
    case 19200:
      dcbSerialParams.BaudRate = CBR_19200;
      break;
#endif
#ifdef CBR_28800
    case 28800:
      dcbSerialParams.BaudRate = CBR_28800;
      break;
#endif
#ifdef CBR_57600
    case 57600:
      dcbSerialParams.BaudRate = CBR_57600;
      break;
#endif
#ifdef CBR_76800
    case 76800:
      dcbSerialParams.BaudRate = CBR_76800;
      break;
#endif
#ifdef CBR_38400
    case 38400:
      dcbSerialParams.BaudRate = CBR_38400;
      break;
#endif
#ifdef CBR_115200
    case 115200:
      dcbSerialParams.BaudRate = CBR_115200;
      break;
#endif
#ifdef CBR_128000
    case 128000:
      dcbSerialParams.BaudRate = CBR_128000;
      break;
#endif
#ifdef CBR_153600
    case 153600:
      dcbSerialParams.BaudRate = CBR_153600;
      break;
#endif
#ifdef CBR_230400
    case 230400:
      dcbSerialParams.BaudRate = CBR_230400;
      break;
#endif
#ifdef CBR_256000
    case 256000:
      dcbSerialParams.BaudRate = CBR_256000;
      break;
#endif
#ifdef CBR_460800
    case 460800:
      dcbSerialParams.BaudRate = CBR_460800;
      break;
#endif
#ifdef CBR_921600
    case 921600:
      dcbSerialParams.BaudRate = CBR_921600;
      break;
#endif
    default:
      // Try to blindly assign it
      dcbSerialParams.BaudRate = baudrate_;
  }

  // setup char len
  if (bytesize_ == eightbits) {
    dcbSerialParams.ByteSize = 8;
  } else if (bytesize_ == sevenbits) {
    dcbSerialParams.ByteSize = 7;
  } else if (bytesize_ == sixbits) {
    dcbSerialParams.ByteSize = 6;
  } else if (bytesize_ == fivebits) {
    dcbSerialParams.ByteSize = 5;
  } else {
    throw SerialInvalidArgumentException("invalid char len");
  }

  // setup stopbits
  if (stopbits_ == stopbits_one) {
    dcbSerialParams.StopBits = ONESTOPBIT;
  } else if (stopbits_ == stopbits_one_point_five) {
    dcbSerialParams.StopBits = ONE5STOPBITS;
  } else if (stopbits_ == stopbits_two) {
    dcbSerialParams.StopBits = TWOSTOPBITS;
  } else {
    throw SerialInvalidArgumentException("invalid stop bit");
  }

  // setup parity
  if (parity_ == parity_none) {
    dcbSerialParams.Parity = NOPARITY;
  } else if (parity_ == parity_even) {
    dcbSerialParams.Parity = EVENPARITY;
  } else if (parity_ == parity_odd) {
    dcbSerialParams.Parity = ODDPARITY;
  } else if (parity_ == parity_mark) {
    dcbSerialParams.Parity = MARKPARITY;
  } else if (parity_ == parity_space) {
    dcbSerialParams.Parity = SPACEPARITY;
  } else {
    throw SerialInvalidArgumentException("invalid parity");
  }

  // setup flowcontrol
  if (flowcontrol_ == flowcontrol_none) {
    dcbSerialParams.fOutxCtsFlow = false;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    dcbSerialParams.fOutX = false;
    dcbSerialParams.fInX = false;
  }
  if (flowcontrol_ == flowcontrol_software) {
    dcbSerialParams.fOutxCtsFlow = false;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    dcbSerialParams.fOutX = true;
    dcbSerialParams.fInX = true;
  }
  if (flowcontrol_ == flowcontrol_hardware) {
    dcbSerialParams.fOutxCtsFlow = true;
    dcbSerialParams.fRtsControl = RTS_CONTROL_HANDSHAKE;
    dcbSerialParams.fOutX = false;
    dcbSerialParams.fInX = false;
  }

  // activate settings
  if (!SetCommState(fd_, &dcbSerialParams)) {
    CloseHandle(fd_);
    throw SerialIOException("error setting serial port settings.");
  }

  // Setup timeouts
  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = timeout_.getInterByteMilliseconds();
  timeouts.ReadTotalTimeoutConstant = timeout_.getReadConstantMilliseconds();
  timeouts.ReadTotalTimeoutMultiplier = timeout_.getReadMultiplierMilliseconds();
  timeouts.WriteTotalTimeoutConstant = timeout_.getWriteConstantMilliseconds();
  timeouts.WriteTotalTimeoutMultiplier = timeout_.getWriteMultiplierMilliseconds();
  if (!SetCommTimeouts(fd_, &timeouts)) {
    throw SerialIOException("error setting timeouts.");
  }
}

void Serial::SerialImpl::close() {
  if (is_open_) {
    if (fd_ != INVALID_HANDLE_VALUE) {
      int ret;
      ret = CloseHandle(fd_);
      if (ret == 0) {
        throw SerialIOException("error '" + std::to_string(GetLastError()) + "' while closing the serial port");
      } else {
        fd_ = INVALID_HANDLE_VALUE;
      }
    }
    is_open_ = false;
  }
}

bool Serial::SerialImpl::isOpen() const {
  return is_open_;
}

size_t Serial::SerialImpl::available() const {
  if (!is_open_) {
    return 0;
  }
  COMSTAT cs;
  if (!ClearCommError(fd_, nullptr, &cs)) {
    throw SerialIOException("error '" + std::to_string(GetLastError()) + "' while checking status of the serial port");
  }
  return static_cast<size_t>(cs.cbInQue);
}

bool Serial::SerialImpl::waitReadable(std::chrono::milliseconds /*timeout*/) {
  throw SerialException("waitReadable() is not implemented on Windows");
}

void Serial::SerialImpl::waitByteTimes(size_t /*count*/) const {
  throw SerialException("waitByteTimes() is not implemented on Windows");
}

size_t Serial::SerialImpl::read(uint8_t *buf, size_t size) {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD bytes_read;
  if (!ReadFile(fd_, buf, static_cast<DWORD>(size), &bytes_read, nullptr)) {
    throw SerialIOException("error '" + std::to_string(GetLastError()) + "' while reading from the serial port");
  }
  return (size_t)(bytes_read);
}

size_t Serial::SerialImpl::write(const uint8_t *data, size_t length) {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD bytes_written;
  if (!WriteFile(fd_, data, static_cast<DWORD>(length), &bytes_written, nullptr)) {
    throw SerialIOException("error '" + std::to_string(GetLastError()) + "' while writing to the serial port");
  }
  return (size_t)(bytes_written);
}

void Serial::SerialImpl::setPort(const std::string &port) {
  port_ = std::wstring(port.begin(), port.end());
}

std::string Serial::SerialImpl::getPort() const {
  return std::string(port_.begin(), port_.end());
}

void Serial::SerialImpl::setTimeout(Serial::Timeout &timeout) {
  timeout_ = timeout;
  if (is_open_) {
    reconfigurePort();
  }
}

Serial::Timeout Serial::SerialImpl::getTimeout() const {
  return timeout_;
}

void Serial::SerialImpl::setBaudrate(unsigned long baudrate) {
  baudrate_ = baudrate;
  if (is_open_) {
    reconfigurePort();
  }
}

unsigned long Serial::SerialImpl::getBaudrate() const {
  return baudrate_;
}

void Serial::SerialImpl::setBytesize(serial::bytesize_t bytesize) {
  bytesize_ = bytesize;
  if (is_open_) {
    reconfigurePort();
  }
}

serial::bytesize_t Serial::SerialImpl::getBytesize() const {
  return bytesize_;
}

void Serial::SerialImpl::setParity(serial::parity_t parity) {
  parity_ = parity;
  if (is_open_) {
    reconfigurePort();
  }
}

serial::parity_t Serial::SerialImpl::getParity() const {
  return parity_;
}

void Serial::SerialImpl::setStopbits(serial::stopbits_t stopbits) {
  stopbits_ = stopbits;
  if (is_open_) {
    reconfigurePort();
  }
}

serial::stopbits_t Serial::SerialImpl::getStopbits() const {
  return stopbits_;
}

void Serial::SerialImpl::setFlowcontrol(serial::flowcontrol_t flowcontrol) {
  flowcontrol_ = flowcontrol;
  if (is_open_) {
    reconfigurePort();
  }
}

serial::flowcontrol_t Serial::SerialImpl::getFlowcontrol() const {
  return flowcontrol_;
}

void Serial::SerialImpl::flush() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  PurgeComm(fd_, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

void Serial::SerialImpl::flushInput() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  PurgeComm(fd_, PURGE_RXCLEAR);
}

void Serial::SerialImpl::flushOutput() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  PurgeComm(fd_, PURGE_TXCLEAR);
}

void Serial::SerialImpl::sendBreak(int /*duration*/) const {
  throw SerialException("sendBreak() is not implemented on Windows");
}

void Serial::SerialImpl::setBreak(bool level) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  EscapeCommFunction(fd_, level ? SETBREAK : CLRBREAK);
}

void Serial::SerialImpl::setRTS(bool level) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  EscapeCommFunction(fd_, level ? SETRTS : CLRRTS);
}

void Serial::SerialImpl::setDTR(bool level) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  EscapeCommFunction(fd_, level ? SETDTR : CLRDTR);
}

bool Serial::SerialImpl::waitForChange() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD dwCommEvent;
  if (!SetCommMask(fd_, EV_CTS | EV_DSR | EV_RING | EV_RLSD)) {
    // Error setting communications mask
    return false;
  }
  return WaitCommEvent(fd_, &dwCommEvent, nullptr) != 0;
}

bool Serial::SerialImpl::getCTS() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD dwModemStatus;
  if (!GetCommModemStatus(fd_, &dwModemStatus)) {
    throw SerialIOException("getCTS() failed on a call to GetCommModemStatus()");
  }

  return (MS_CTS_ON & dwModemStatus) != 0;
}

bool Serial::SerialImpl::getDSR() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD dwModemStatus;
  if (!GetCommModemStatus(fd_, &dwModemStatus)) {
    throw SerialIOException("getDSR() failed on a call to GetCommModemStatus()");
  }
  return (MS_DSR_ON & dwModemStatus) != 0;
}

bool Serial::SerialImpl::getRI() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD dwModemStatus;
  if (!GetCommModemStatus(fd_, &dwModemStatus)) {
    throw SerialIOException("getRI() failed on a call to GetCommModemStatus()");
  }
  return (MS_RING_ON & dwModemStatus) != 0;
}

bool Serial::SerialImpl::getCD() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  DWORD dwModemStatus;
  if (!GetCommModemStatus(fd_, &dwModemStatus)) {
    throw SerialIOException("getCD() failed on a call to GetCommModemStatus()");
  }
  return (MS_RLSD_ON & dwModemStatus) != 0;
}

#endif  // defined(_WIN32)
