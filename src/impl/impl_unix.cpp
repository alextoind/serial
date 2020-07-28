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

#if !defined(_WIN32)

#include <serial/impl/impl.h>

using namespace serial;

template<typename T>
timespec getTimeSpec(std::chrono::duration<int64_t, T> duration) {
  using namespace std::chrono;
  return {duration_cast<seconds>(duration).count(), (duration_cast<nanoseconds>(duration) - duration_cast<seconds>(duration)).count()};
}

Serial::SerialImpl::SerialImpl(std::string port, unsigned long baudrate, Timeout timeout, bytesize_t bytesize,
                               parity_t parity, stopbits_t stopbits, flowcontrol_t flowcontrol)
    : port_(std::move(port)),
      fd_(-1),
      byte_time_ns_(0),
      is_open_(false),
      xonxoff_(false),
      rtscts_(false),
      timeout_(timeout),
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

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ == -1) {
    switch (errno) {
      case EINTR:
        // Recurse because this is a recoverable error.
        open();
        return;
      case ENFILE:
      case EMFILE:
        throw SerialIOException("too many file handles open.");
      default:
        throw SerialIOException();
    }
  }

  reconfigurePort();
  is_open_ = true;
}

void Serial::SerialImpl::reconfigurePort() {
  if (fd_ == -1) {
    // Can only operate on a valid file descriptor
    throw SerialIOException("invalid file descriptor, is the serial port open?");
  }

  struct termios options; // The options for the file descriptor

  if (tcgetattr(fd_, &options) == -1) {
    throw SerialIOException("tcgetattr() fails during reconfigurePort()");
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t)~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN); //|ECHOPRT

  options.c_oflag &= (tcflag_t)~(OPOST);
  options.c_iflag &= (tcflag_t)~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
  options.c_iflag &= (tcflag_t)~IUCLC;
#endif
#ifdef PARMRK
  options.c_iflag &= (tcflag_t)~PARMRK;
#endif

  // setup baud rate
  bool custom_baud = false;
  speed_t baud;
  switch (baudrate_) {
#ifdef B0
    case 0:
      baud = B0;
      break;
#endif
#ifdef B50
    case 50:
      baud = B50;
      break;
#endif
#ifdef B75
    case 75:
      baud = B75;
      break;
#endif
#ifdef B110
    case 110:
      baud = B110;
      break;
#endif
#ifdef B134
    case 134:
      baud = B134;
      break;
#endif
#ifdef B150
    case 150:
      baud = B150;
      break;
#endif
#ifdef B200
    case 200:
      baud = B200;
      break;
#endif
#ifdef B300
    case 300:
      baud = B300;
      break;
#endif
#ifdef B600
    case 600:
      baud = B600;
      break;
#endif
#ifdef B1200
    case 1200:
      baud = B1200;
      break;
#endif
#ifdef B1800
    case 1800:
      baud = B1800;
      break;
#endif
#ifdef B2400
    case 2400:
      baud = B2400;
      break;
#endif
#ifdef B4800
    case 4800:
      baud = B4800;
      break;
#endif
#ifdef B7200
    case 7200:
      baud = B7200;
      break;
#endif
#ifdef B9600
    case 9600:
      baud = B9600;
      break;
#endif
#ifdef B14400
    case 14400:
      baud = B14400;
      break;
#endif
#ifdef B19200
    case 19200:
      baud = B19200;
      break;
#endif
#ifdef B28800
    case 28800:
      baud = B28800;
      break;
#endif
#ifdef B57600
    case 57600:
      baud = B57600;
      break;
#endif
#ifdef B76800
    case 76800:
      baud = B76800;
      break;
#endif
#ifdef B38400
    case 38400:
      baud = B38400;
      break;
#endif
#ifdef B115200
    case 115200:
      baud = B115200;
      break;
#endif
#ifdef B128000
    case 128000:
      baud = B128000;
      break;
#endif
#ifdef B153600
    case 153600:
      baud = B153600;
      break;
#endif
#ifdef B230400
    case 230400:
      baud = B230400;
      break;
#endif
#ifdef B256000
    case 256000:
      baud = B256000;
      break;
#endif
#ifdef B460800
    case 460800:
      baud = B460800;
      break;
#endif
#ifdef B500000
    case 500000:
      baud = B500000;
      break;
#endif
#ifdef B576000
    case 576000:
      baud = B576000;
      break;
#endif
#ifdef B921600
    case 921600:
      baud = B921600;
      break;
#endif
#ifdef B1000000
    case 1000000:
      baud = B1000000;
      break;
#endif
#ifdef B1152000
    case 1152000:
      baud = B1152000;
      break;
#endif
#ifdef B1500000
    case 1500000:
      baud = B1500000;
      break;
#endif
#ifdef B2000000
    case 2000000:
      baud = B2000000;
      break;
#endif
#ifdef B2500000
  case 2500000:
    baud = B2500000;
    break;
#endif
#ifdef B3000000
    case 3000000:
      baud = B3000000;
      break;
#endif
#ifdef B3500000
    case 3500000:
      baud = B3500000;
      break;
#endif
#ifdef B4000000
    case 4000000:
      baud = B4000000;
      break;
#endif
    default:
      custom_baud = true;
      // OS X support
#if defined(MAC_OS_X_VERSION_10_4) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_4)
      // Starting with Tiger, the IOSSIOSPEED ioctl can be used to set arbitrary baud rates
      // other than those specified by POSIX. The driver for the underlying serial hardware
      // ultimately determines which baud rates can be used. This ioctl sets both the input
      // and output speed.
      speed_t new_baud = static_cast<speed_t> (baudrate_);
      if (-1 == ioctl (fd_, IOSSIOSPEED, &new_baud, 1)) {
        throw SerialIOException();
      }
      // Linux Support
#elif defined(__linux__) && defined (TIOCSSERIAL)
      struct serial_struct ser;

      if (-1 == ioctl(fd_, TIOCGSERIAL, &ser)) {
        throw SerialIOException();
      }

      // set custom divisor
      ser.custom_divisor = ser.baud_base / static_cast<int> (baudrate_);
      // update flags
      ser.flags &= ~ASYNC_SPD_MASK;
      ser.flags |= ASYNC_SPD_CUST;

      if (-1 == ioctl(fd_, TIOCSSERIAL, &ser)) {
        throw SerialIOException();
      }
#else
      throw SerialInvalidArgumentException("OS does not currently support custom bauds");
#endif
  }
  if (!custom_baud) {
#ifdef _BSD_SOURCE
    ::cfsetspeed(&options, baud);
#else
    ::cfsetispeed(&options, baud);
    ::cfsetospeed(&options, baud);
#endif
  }

  // setup char len
  options.c_cflag &= (tcflag_t)~CSIZE;
  if (bytesize_ == eightbits) {
    options.c_cflag |= CS8;
  } else if (bytesize_ == sevenbits) {
    options.c_cflag |= CS7;
  } else if (bytesize_ == sixbits) {
    options.c_cflag |= CS6;
  } else if (bytesize_ == fivebits) {
    options.c_cflag |= CS5;
  } else {
    throw SerialInvalidArgumentException("invalid char len");
  }
  // setup stopbits
  if (stopbits_ == stopbits_one) {
    options.c_cflag &= (tcflag_t)~(CSTOPB);
  } else if (stopbits_ == stopbits_one_point_five || stopbits_ == stopbits_two) {
    // ONE POINT FIVE same as TWO.. there is no POSIX support for 1.5
    options.c_cflag |= (CSTOPB);
  } else {
    throw SerialInvalidArgumentException("invalid stop bit");
  }
  // setup parity
  options.c_iflag &= (tcflag_t)~(INPCK | ISTRIP);
  if (parity_ == parity_none) {
    options.c_cflag &= (tcflag_t)~(PARENB | PARODD);
  } else if (parity_ == parity_even) {
    options.c_cflag &= (tcflag_t)~(PARODD);
    options.c_cflag |= (PARENB);
  } else if (parity_ == parity_odd) {
    options.c_cflag |= (PARENB | PARODD);
  }
#ifdef CMSPAR
  else if (parity_ == parity_mark) {
    options.c_cflag |= (PARENB | CMSPAR | PARODD);
  } else if (parity_ == parity_space) {
    options.c_cflag |= (PARENB | CMSPAR);
    options.c_cflag &= (tcflag_t)~(PARODD);
  }
#else
  // CMSPAR is not defined on OSX. So do not support mark or space parity.
  else if (parity_ == parity_mark || parity_ == parity_space) {
    throw SerialInvalidArgumentException("OS does not support mark or space parity");
  }
#endif  // ifdef CMSPAR
  else {
    throw SerialInvalidArgumentException("invalid parity");
  }
  // setup flow control
  if (flowcontrol_ == flowcontrol_none) {
    xonxoff_ = false;
    rtscts_ = false;
  }
  if (flowcontrol_ == flowcontrol_software) {
    xonxoff_ = true;
    rtscts_ = false;
  }
  if (flowcontrol_ == flowcontrol_hardware) {
    xonxoff_ = false;
    rtscts_ = true;
  }
  // xonxoff
#ifdef IXANY
  if (xonxoff_) {
    options.c_iflag |= (IXON | IXOFF); //|IXANY)
  } else {
    options.c_iflag &= (tcflag_t)~(IXON | IXOFF | IXANY);
  }
#else
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF);
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif
  // rtscts
#ifdef CRTSCTS
  if (rtscts_) {
    options.c_cflag |= (CRTSCTS);
  } else {
    options.c_cflag &= (unsigned long)~(CRTSCTS);
  }
#elif defined CNEW_RTSCTS
  if (rtscts_)
    options.c_cflag |=  (CNEW_RTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CNEW_RTSCTS);
#else
#error "OS Support seems wrong."
#endif

  // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
  // this basically sets the read call up to be a polling read,
  // but we are using select to ensure there is data available
  // to read before each call, so we should never needlessly poll
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // activate settings
  ::tcsetattr(fd_, TCSANOW, &options);

  // Update byte_time_ based on the new settings.
  uint32_t bit_time_ns = 1e9 / baudrate_;
  byte_time_ns_ = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);

  // Compensate for the stopbits_one_point_five enum being equal to int 3,
  // and not 1.5.
  if (stopbits_ == stopbits_one_point_five) {
    byte_time_ns_ += ((1.5 - stopbits_one_point_five) * bit_time_ns);
  }
}

void Serial::SerialImpl::close() {
  if (is_open_) {
    if (fd_ != -1) {
      int ret;
      ret = ::close(fd_);
      if (ret == 0) {
        fd_ = -1;
      } else {
        throw SerialIOException();
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
  int count = 0;
  if (-1 == ioctl(fd_, TIOCINQ, &count)) {
    throw SerialIOException();
  } else {
    return static_cast<size_t> (count);
  }
}

bool Serial::SerialImpl::waitReadable(std::chrono::milliseconds timeout_ms) {
  // Setup a select call to block for serial data or a timeout
  fd_set readfds;
  FD_ZERO (&readfds);
  FD_SET (fd_, &readfds);
  timespec timeout = getTimeSpec(timeout_ms);
  int r = pselect(fd_ + 1, &readfds, nullptr, nullptr, &timeout, nullptr);

  if (r < 0) {
    // Select was interrupted
    if (errno == EINTR) {
      return false;
    }
    // Otherwise there was some error
    throw SerialIOException();
  }
  // Timeout occurred
  if (r == 0) {
    return false;
  }
  // This shouldn't happen, if r > 0 our fd has to be in the list!
  if (!FD_ISSET (fd_, &readfds)) {
    throw SerialIOException("select reports ready to read, but our fd isn't in the list, this shouldn't happen!");
  }
  // Data available to read.
  return true;
}

void Serial::SerialImpl::waitByteTimes(size_t count) const {
  timespec wait_time = {0, static_cast<long>(byte_time_ns_ * count)};
  pselect(0, nullptr, nullptr, nullptr, &wait_time, nullptr);
}

size_t Serial::SerialImpl::read(uint8_t *buf, size_t size) {
  // If the port is not open, throw
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  size_t bytes_read = 0;

  auto read_deadline = timeout_.getReadDeadline(size);

  // Pre-fill buffer with available bytes
  {
    ssize_t bytes_read_now = ::read(fd_, buf, size);
    if (bytes_read_now > 0) {
      bytes_read = bytes_read_now;
    }
  }

  while (bytes_read < size) {
    auto remaining_time = Timeout::remainingMilliseconds(read_deadline);
    if (remaining_time.count() <= 0) {
      // Timed out
      break;
    }
    // Timeout for the next select is whichever is less of the remaining
    // total read timeout and the inter-byte timeout.
    // Wait for the device to be readable, and then attempt to read.
    if (waitReadable(std::min(remaining_time, timeout_.getInterByte()))) {
      // If it's a fixed-length multi-byte read, insert a wait here so that
      // we can attempt to grab the whole thing in a single IO call. Skip
      // this wait if a non-max inter_byte_timeout is specified.
      if (size > 1 && timeout_.getInterByteMilliseconds() == std::numeric_limits<uint32_t>::max()) {
        size_t bytes_available = available();
        if (bytes_available + bytes_read < size) {
          waitByteTimes(size - (bytes_available + bytes_read));
        }
      }
      // This should be non-blocking returning only what is available now
      //  Then returning so that select can block again.
      ssize_t bytes_read_now = ::read(fd_, buf + bytes_read, size - bytes_read);
      // read should always return some data as select reported it was
      // ready to read when we get to this point.
      if (bytes_read_now < 1) {
        // Disconnected devices, at least on Linux, show the
        // behavior that they are always ready to read immediately
        // but reading returns nothing.
        throw SerialException("device reports readiness to read but returned no data (device disconnected?)");
      }
      // Update bytes_read
      bytes_read += static_cast<size_t> (bytes_read_now);
      // If bytes_read == size then we have read everything we need
      if (bytes_read == size) {
        break;
      }
      // If bytes_read < size then we have more to read
      if (bytes_read < size) {
        continue;
      }
      // If bytes_read > size then we have over read, which shouldn't happen
      if (bytes_read > size) {
        throw SerialException("read over read, too many bytes where read, this shouldn't happen, might be a logical error!");
      }
    }
  }
  return bytes_read;
}

size_t Serial::SerialImpl::write(const uint8_t *data, size_t length) {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  fd_set writefds;
  size_t bytes_written = 0;

  auto write_deadline = timeout_.getWriteDeadline(length);

  bool first_iteration = true;
  while (bytes_written < length) {
    auto remaining_time = Timeout::remainingMilliseconds(write_deadline);
    // Only consider the timeout if it's not the first iteration of the loop
    // otherwise a timeout of 0 won't be allowed through
    if (!first_iteration && (remaining_time.count() <= 0)) {
      // Timed out
      break;
    }
    first_iteration = false;

    FD_ZERO (&writefds);
    FD_SET (fd_, &writefds);
    timespec timeout = getTimeSpec(remaining_time);
    int r = pselect(fd_ + 1, nullptr, &writefds, nullptr, &timeout, nullptr);

    // Figure out what happened by looking at select's response 'r'
    /** Error **/
    if (r < 0) {
      // Select was interrupted, try again
      if (errno == EINTR) {
        continue;
      }
      // Otherwise there was some error
      throw SerialIOException();
    }
    /** Timeout **/
    if (r == 0) {
      break;
    }
    /** Port ready to write **/
    if (r > 0) {
      // Make sure our file descriptor is in the ready to write list
      if (FD_ISSET (fd_, &writefds)) {
        // This will write some
        ssize_t bytes_written_now = ::write(fd_, data + bytes_written, length - bytes_written);
        // write should always return some data as select reported it was
        // ready to write when we get to this point.
        if (bytes_written_now < 1) {
          // Disconnected devices, at least on Linux, show the
          // behavior that they are always ready to write immediately
          // but writing returns nothing.
          throw SerialException("device reports readiness to write but returned no data (device disconnected?)");
        }
        // Update bytes_written
        bytes_written += static_cast<size_t> (bytes_written_now);
        // If bytes_written == size then we have written everything we need to
        if (bytes_written == length) {
          break;
        }
        // If bytes_written < size then we have more to write
        if (bytes_written < length) {
          continue;
        }
        // If bytes_written > size then we have over written, which shouldn't happen
        if (bytes_written > length) {
          throw SerialException("write over wrote, too many bytes where written, this shouldn't happen, might be a logical error!");
        }
      }
      // This shouldn't happen, if r > 0 our fd has to be in the list!
      throw SerialIOException("select reports ready to write, but our fd isn't in the list, this shouldn't happen!");
    }
  }
  return bytes_written;
}

void Serial::SerialImpl::setPort(const std::string &port) {
  port_ = port;
}

std::string Serial::SerialImpl::getPort() const {
  return port_;
}

void Serial::SerialImpl::setTimeout(Serial::Timeout &timeout) {
  timeout_ = timeout;  // timeout is used directly inside read() and write(): there is no need to call reconfigurePort()
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
  if (::tcflush(fd_, TCIOFLUSH) == -1) {
    throw SerialIOException("failure during ::tcflush()", errno);
  }
}

void Serial::SerialImpl::flushInput() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::tcflush(fd_, TCIFLUSH) == -1) {
    throw SerialIOException("failure during ::tcflush()", errno);
  }
}

void Serial::SerialImpl::flushOutput() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::tcflush(fd_, TCOFLUSH) == -1) {
    throw SerialIOException("failure during ::tcflush()", errno);
  }
}

void Serial::SerialImpl::sendBreak(int duration_ms) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::tcsendbreak(fd_, duration_ms) == -1) {
    throw SerialIOException("failure during ::tcsendbreak()", errno);
  }
}

void Serial::SerialImpl::setBreak(bool level) const {
  setModemStatus(level ? TIOCSBRK : TIOCCBRK);
}

void Serial::SerialImpl::setModemStatus(uint32_t request, uint32_t command) const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  if (::ioctl(fd_, request, &command) == -1) {
    throw SerialIOException("failure during ::ioctl()", errno);
  }
}

void Serial::SerialImpl::setRTS(bool level) const {
  setModemStatus(level ? TIOCMBIS : TIOCMBIC, TIOCM_RTS);
}

void Serial::SerialImpl::setDTR(bool level) const {
  setModemStatus(level ? TIOCMBIS : TIOCMBIC, TIOCM_DTR);
}

void Serial::SerialImpl::waitForModemChanges() const {
#ifndef TIOCMIWAIT
  throw SerialException("TIOCMIWAIT is not defined");
#else
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  // cannot use setModemStatus(): TIOCMIWAIT requires arg by value (not by pointer)
  if (::ioctl(fd_, TIOCMIWAIT, TIOCM_CTS | TIOCM_DSR | TIOCM_RI | TIOCM_CD) == -1) {
    throw SerialIOException("failure during ::ioctl()", errno);
  }
#endif
}

uint32_t Serial::SerialImpl::getModemStatus() const {
  if (!is_open_) {
    throw SerialPortNotOpenException();
  }
  uint32_t modem_status;
  if (::ioctl(fd_, TIOCMGET, &modem_status) == -1) {
    throw SerialIOException("failure during ::ioctl()", errno);
  }
  return modem_status;
}

bool Serial::SerialImpl::getCTS() const {
  return getModemStatus() & TIOCM_CTS;
}

bool Serial::SerialImpl::getDSR() const {
  return getModemStatus() & TIOCM_DSR;
}

bool Serial::SerialImpl::getRI() const {
  return getModemStatus() & TIOCM_RI;
}

bool Serial::SerialImpl::getCD() const {
  return getModemStatus() & TIOCM_CD;
}

#endif  // !defined(_WIN32)
