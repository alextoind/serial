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

#ifndef SERIAL_H
#define SERIAL_H

#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace serial {

/*!
 * Enumeration defines the possible bytesizes for the serial port.
 */
typedef enum {
  fivebits = 5,
  sixbits = 6,
  sevenbits = 7,
  eightbits = 8
} bytesize_t;

/*!
 * Enumeration defines the possible parity types for the serial port.
 */
typedef enum {
  parity_none = 0,
  parity_odd = 1,
  parity_even = 2,
  parity_mark = 3,
  parity_space = 4
} parity_t;

/*!
 * Enumeration defines the possible stopbit types for the serial port.
 */
typedef enum {
  stopbits_one = 1,
  stopbits_two = 2,
  stopbits_one_point_five
} stopbits_t;

/*!
 * Enumeration defines the possible flowcontrol types for the serial port.
 */
typedef enum {
  flowcontrol_none = 0,
  flowcontrol_software,
  flowcontrol_hardware
} flowcontrol_t;

/*!
 * Class that provides a portable serial port interface.
 */
class Serial {
 public:
  class Timeout {
   public:
    Timeout() = default;
    ~Timeout() = default;

    explicit Timeout(uint32_t read_write_constant)
    // the ugly parentheses around `(std::max)()` are required on Windows (this is to avoid to `#undef max` or to `#define NOMINMAX`)
        : Timeout((std::numeric_limits<uint32_t>::max)(), read_write_constant, 0, read_write_constant, 0) {}

    explicit Timeout(uint32_t inter_byte, uint32_t read_constant, uint32_t read_multiplier, uint32_t write_constant, uint32_t write_multiplier)
        : inter_byte_(inter_byte),
          read_constant_(read_constant),
          read_multiplier_(read_multiplier),
          write_constant_(write_constant),
          write_multiplier_(write_multiplier) {}

    std::chrono::milliseconds getInterByte() { return inter_byte_; }
    uint32_t getInterByteMilliseconds() { return inter_byte_.count(); }
    std::chrono::milliseconds getReadConstant() { return read_constant_; }
    uint32_t getReadConstantMilliseconds() { return read_constant_.count(); }
    std::chrono::milliseconds getReadMultiplier() { return read_multiplier_; }
    uint32_t getReadMultiplierMilliseconds() { return read_multiplier_.count(); }
    std::chrono::milliseconds getWriteConstant() { return write_constant_; }
    uint32_t getWriteConstantMilliseconds() { return write_constant_.count(); }
    std::chrono::milliseconds getWriteMultiplier() { return write_multiplier_; }
    uint32_t getWriteMultiplierMilliseconds() { return write_multiplier_.count(); }

    std::chrono::steady_clock::time_point getReadDeadline() { return getReadDeadline(0); }
    std::chrono::steady_clock::time_point getReadDeadline(size_t size) {
      return std::chrono::steady_clock::now() + read_constant_ + read_multiplier_*size;
    }

    std::chrono::steady_clock::time_point getWriteDeadline() { return getWriteDeadline(0); }
    std::chrono::steady_clock::time_point getWriteDeadline(size_t size) {
      return std::chrono::steady_clock::now() + write_constant_ + write_multiplier_*size;
    }

    static std::chrono::milliseconds remainingMilliseconds(std::chrono::steady_clock::time_point deadline) {
      return std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    }

   private:
    // the following timeouts in milliseconds refer to COMMTIMEOUTS (https://docs.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-commtimeouts)
    std::chrono::duration<uint32_t, std::milli> inter_byte_;
    std::chrono::duration<uint32_t, std::milli> read_constant_;
    std::chrono::duration<uint32_t, std::milli> read_multiplier_;
    std::chrono::duration<uint32_t, std::milli> write_constant_;
    std::chrono::duration<uint32_t, std::milli> write_multiplier_;
  };

  /*!
   * Creates a Serial object and opens the port if a port is specified,
   * otherwise it remains closed until serial::Serial::open is called.
   *
   * \param port A std::string containing the address of the serial port,
   *        which would be something like 'COM1' on Windows and '/dev/ttyS0'
   *        on Linux.
   *
   * \param baudrate An unsigned 32-bit integer that represents the baudrate
   *
   * \param timeout A serial::Timeout struct that defines the timeout
   * conditions for the serial port. \see serial::Timeout
   *
   * \param bytesize Size of each byte in the serial transmission of data,
   * default is eightbits, possible values are: fivebits, sixbits, sevenbits,
   * eightbits
   *
   * \param parity Method of parity, default is parity_none, possible values
   * are: parity_none, parity_odd, parity_even
   *
   * \param stopbits Number of stop bits used, default is stopbits_one,
   * possible values are: stopbits_one, stopbits_one_point_five, stopbits_two
   *
   * \param flowcontrol Type of flowcontrol used, default is
   * flowcontrol_none, possible values are: flowcontrol_none,
   * flowcontrol_software, flowcontrol_hardware
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   * \throw serial::SerialPortNotOpenException
   */
  explicit Serial(const std::string &port = "", uint32_t baudrate = 9600, Timeout timeout = Timeout(),
                  bytesize_t bytesize = eightbits, parity_t parity = parity_none, stopbits_t stopbits = stopbits_one,
                  flowcontrol_t flowcontrol = flowcontrol_none);

  /*! Destructor */
  virtual ~Serial();

  Serial(const Serial &) = delete;
  Serial &operator=(const Serial &) = delete;

  /*!
   * Opens the serial port as long as the port is set and the port isn't
   * already open.
   *
   * If the port is provided to the constructor then an explicit call to open
   * is not needed.
   *
   * \see Serial::Serial
   *
   * \throw serial::SerialException
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void open();

  /*! Gets the open status of the serial port.
   *
   * \return Returns true if the port is open, false otherwise.
   */
  bool isOpen() const;

  /*! Closes the serial port.
   *
   * \throw serial::SerialIOException
   */
  void close();

  /*! Return the number of characters in the buffer.
   *
   * \throw serial::SerialIOException
   */
  size_t available();

  /*! Block until there is serial data to read or read_timeout_constant
   * number of milliseconds have elapsed. The return value is true when
   * the function exits with the port in a readable state, false otherwise
   * (due to timeout or select interruption).
   * Implemented only on Unix.
   *
   * \throw serial::SerialIOException
   */
  bool waitReadable();

  /*! Block for a period of time corresponding to the transmission time of
   * count characters at present serial settings. This may be used in con-
   * junction with waitReadable to read larger blocks of data from the
   * port.
   * Implemented only on Unix.
   */
  void waitByteTimes(size_t count);

  /*! Read a given amount of bytes from the serial port into a given buffer.
   *
   * The read function will return in one of three cases:
   *  * The number of requested bytes was read.
   *    * In this case the number of bytes requested will match the size_t
   *      returned by read.
   *  * A timeout occurred, in this case the number of bytes read will not
   *    match the amount requested, but no exception will be thrown.  One of
   *    two possible timeouts occurred:
   *    * The inter byte timeout expired, this means that number of
   *      milliseconds elapsed between receiving bytes from the serial port
   *      exceeded the inter byte timeout.
   *    * The total timeout expired, which is calculated by multiplying the
   *      read timeout multiplier by the number of requested bytes and then
   *      added to the read timeout constant.  If that total number of
   *      milliseconds elapses after the initial call to read a timeout will
   *      occur.
   *  * An exception occurred, in this case an actual exception will be thrown.
   *
   * \param buffer An uint8_t array of at least the requested size.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException  TODO: actually thrown only by Windows
   * \throw serial::SerialPortNotOpenException
   */
  size_t read(uint8_t *buffer, size_t size);

  /*! Read a given amount of bytes from the serial port into a give buffer.
   *
   * \param buffer A reference to a std::vector of uint8_t.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException  TODO: actually thrown only by Windows
   * \throw serial::SerialPortNotOpenException
   */
  size_t read(std::vector<uint8_t> &buffer, size_t size = 1);

  /*! Read a given amount of bytes from the serial port into a give buffer.
   *
   * \param buffer A reference to a std::string.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException  TODO: actually thrown only by Windows
   * \throw serial::SerialPortNotOpenException
   */
  size_t read(std::string &buffer, size_t size = 1);

  /*! Read a given amount of bytes from the serial port and return a string
   *  containing the data.
   *
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A std::string containing the data read from the port.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException  TODO: actually thrown only by Windows
   * \throw serial::SerialPortNotOpenException
   */
  std::string read(size_t size = 1);

  /*! Reads in a line or until a given delimiter has been processed.
   *
   * Reads from the serial port until a single line has been read.
   *
   * \param line A std::string reference used to store the data.
   * \param size A maximum length of a line, defaults to 65536 (2^16)
   * \param eol A string to match against for the EOL.
   *
   * \return A size_t representing the number of bytes read.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException  TODO: actually thrown only by Windows
   * \throw serial::SerialPortNotOpenException
   */
  size_t readline(std::string &line, size_t size = 65536, const std::string &eol = "\n");

  /*! Reads in a line or until a given delimiter has been processed.
   *
   * Reads from the serial port until a single line has been read.
   *
   * \param size A maximum length of a line, defaults to 65536 (2^16)
   * \param eol A string to match against for the EOL.
   *
   * \return A std::string containing the line.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException  TODO: actually thrown only by Windows
   * \throw serial::SerialPortNotOpenException
   */
  std::string readline(size_t size = 65536, const std::string &eol = "\n");

  /*! Reads in multiple lines until the serial port times out.
   *
   * This requires a timeout > 0 before it can be run. It will read until a
   * timeout occurs and return a list of strings.
   *
   * \param size A maximum length of combined lines, defaults to 65536 (2^16)
   *
   * \param eol A string to match against for the EOL.
   *
   * \return A vector<string> containing the lines.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException  TODO: actually thrown only by Windows
   * \throw serial::SerialPortNotOpenException
   */
  std::vector<std::string> readlines(size_t size = 65536, const std::string &eol = "\n");

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \param size A size_t that indicates how many bytes should be written from
   * the given data buffer.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException
   * \throw serial::SerialPortNotOpenException
   */
  size_t write(const uint8_t *data, size_t size);

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException
   * \throw serial::SerialPortNotOpenException
   */
  size_t write(const std::vector<uint8_t> &data);

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::SerialException  TODO: actually thrown only by Unix
   * \throw serial::SerialIOException
   * \throw serial::SerialPortNotOpenException
   */
  size_t write(const std::string &data);

  /*! Sets the serial port identifier.
   *
   * \param port A const std::string reference containing the address of the
   * serial port, which would be something like 'COM1' on Windows and
   * '/dev/ttyS0' on Linux.
   *
   * \throw serial::SerialException
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void setPort(const std::string &port);

  /*! Gets the serial port identifier.
   *
   * \see Serial::setPort
   */
  std::string getPort() const;

  /*! Sets the timeout for reads and writes using the Timeout struct.
   *
   * There are two timeout conditions described here:
   *  * The inter byte timeout:
   *    * The inter_byte_timeout component of serial::Timeout defines the
   *      maximum amount of time, in milliseconds, between receiving bytes on
   *      the serial port that can pass before a timeout occurs.  Setting this
   *      to zero will prevent inter byte timeouts from occurring.
   *  * Total time timeout:
   *    * The constant and multiplier component of this timeout condition,
   *      for both read and write, are defined in serial::Timeout.  This
   *      timeout occurs if the total time since the read or write call was
   *      made exceeds the specified time in milliseconds.
   *    * The limit is defined by multiplying the multiplier component by the
   *      number of requested bytes and adding that product to the constant
   *      component.  In this way if you want a read call, for example, to
   *      timeout after exactly one second regardless of the number of bytes
   *      you asked for then set the read_timeout_constant component of
   *      serial::Timeout to 1000 and the read_timeout_multiplier to zero.
   *      This timeout condition can be used in conjunction with the inter
   *      byte timeout condition with out any problems, timeout will simply
   *      occur when one of the two timeout conditions is met.  This allows
   *      users to have maximum control over the trade-off between
   *      responsiveness and efficiency.
   *
   * Read and write functions will return in one of three cases.  When the
   * reading or writing is complete, when a timeout occurs, or when an
   * exception occurs.
   *
   * A timeout of 0 enables non-blocking mode.
   *
   * \param timeout A serial::Timeout struct containing the inter byte
   * timeout, and the read and write timeout constants and multipliers.
   *
   * \see serial::Timeout
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void setTimeout(Timeout &timeout);

  /*! Sets the timeout for reads and writes.
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void setTimeout(uint32_t inter_byte_timeout, uint32_t read_timeout_constant, uint32_t read_timeout_multiplier,
                  uint32_t write_timeout_constant, uint32_t write_timeout_multiplier) {
    Timeout timeout(inter_byte_timeout, read_timeout_constant, read_timeout_multiplier, write_timeout_constant,
                    write_timeout_multiplier);
    return setTimeout(timeout);
  }

  /*! Gets the timeout for reads in seconds.
   *
   * \return A Timeout struct containing the inter_byte_timeout, and read
   * and write timeout constants and multipliers.
   *
   * \see Serial::setTimeout
   */
  Timeout getTimeout() const;

  /*! Sets the baudrate for the serial port.
   *
   * Possible baudrates depends on the system but some safe baudrates include:
   * 110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000,
   * 57600, 115200
   * Some other baudrates that are supported by some comports:
   * 128000, 153600, 230400, 256000, 460800, 500000, 921600
   *
   * \param baudrate An integer that sets the baud rate for the serial port.
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void setBaudrate(uint32_t baudrate);

  /*! Gets the baudrate for the serial port.
   *
   * \return An integer that sets the baud rate for the serial port.
   *
   * \see Serial::setBaudrate
   */
  uint32_t getBaudrate() const;

  /*! Sets the bytesize for the serial port.
   *
   * \param bytesize Size of each byte in the serial transmission of data,
   * default is eightbits, possible values are: fivebits, sixbits, sevenbits,
   * eightbits
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void setBytesize(bytesize_t bytesize);

  /*! Gets the bytesize for the serial port.
   *
   * \see Serial::setBytesize
   */
  bytesize_t getBytesize() const;

  /*! Sets the parity for the serial port.
   *
   * \param parity Method of parity, default is parity_none, possible values
   * are: parity_none, parity_odd, parity_even
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void setParity(parity_t parity);

  /*! Gets the parity for the serial port.
   *
   * \see Serial::setParity
   */
  parity_t getParity() const;

  /*! Sets the stopbits for the serial port.
   *
   * \param stopbits Number of stop bits used, default is stopbits_one,
   * possible values are: stopbits_one, stopbits_one_point_five, stopbits_two
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void setStopbits(stopbits_t stopbits);

  /*! Gets the stopbits for the serial port.
   *
   * \see Serial::setStopbits
   */
  stopbits_t getStopbits() const;

  /*! Sets the flow control for the serial port.
   *
   * \param flowcontrol Type of flowcontrol used, default is flowcontrol_none,
   * possible values are: flowcontrol_none, flowcontrol_software,
   * flowcontrol_hardware
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialInvalidArgumentException
   */
  void setFlowcontrol(flowcontrol_t flowcontrol);

  /*! Gets the flow control for the serial port.
   *
   * \see Serial::setFlowcontrol
   */
  flowcontrol_t getFlowcontrol() const;

  /*! Flush the input and output buffers
   *
   * \throw serial::SerialPortNotOpenException
   */
  void flush();

  /*! Flush only the input buffer
   *
   * \throw serial::SerialPortNotOpenException
   */
  void flushInput();

  /*! Flush only the output buffer
   *
   * \throw serial::SerialPortNotOpenException
   */
  void flushOutput();

  /*! Sends the RS-232 break signal.  See tcsendbreak(3).
   *  Implemented only on Unix.
   *
   * \throw serial::SerialPortNotOpenException
   */
  void sendBreak(int duration);

  /*! Set the break condition to a given level.  Defaults to true.
   *
   * \throw serial::SerialIOException  TODO: actually thrown only by Unix
   * \throw serial::SerialPortNotOpenException
   */
  void setBreak(bool level = true);

  /*! Set the RTS handshaking line to the given level.  Defaults to true.
   *
   * \throw serial::SerialIOException  TODO: actually thrown only by Unix
   * \throw serial::SerialPortNotOpenException
   */
  void setRTS(bool level = true);

  /*! Set the DTR handshaking line to the given level.  Defaults to true.
   *
   * \throw serial::SerialIOException  TODO: actually thrown only by Unix
   * \throw serial::SerialPortNotOpenException
   */
  void setDTR(bool level = true);

  /*!
   * Blocks until CTS, DSR, RI, CD changes or something interrupts it.
   *
   * Can throw an exception if an error occurs while waiting.
   * You can check the status of CTS, DSR, RI, and CD once this returns.
   * Uses TIOCMIWAIT via ioctl if available (mostly only on Linux) with a
   * resolution of less than +-1ms and as good as +-0.2ms.  Otherwise a
   * polling method is used which can give +-2ms.
   *
   * \return Returns true if one of the lines changed, false if something else
   * occurred.
   *
   * \throw serial::SerialIOException  TODO: actually thrown only by Unix
   * \throw serial::SerialPortNotOpenException  TODO: actually thrown only by Windows
   */
  bool waitForChange();

  /*! Returns the current status of the CTS line.
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialPortNotOpenException
   */
  bool getCTS();

  /*! Returns the current status of the DSR line.
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialPortNotOpenException
   */
  bool getDSR();

  /*! Returns the current status of the RI line.
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialPortNotOpenException
   */
  bool getRI();

  /*! Returns the current status of the CD line.
   *
   * \throw serial::SerialIOException
   * \throw serial::SerialPortNotOpenException
   */
  bool getCD();

 private:
  class SerialImpl;
  std::unique_ptr<SerialImpl> pimpl_;
  std::mutex read_mutex_;
  std::mutex write_mutex_;

  size_t readline_(std::string &line, size_t size = 65536, const std::string &eol = "\n");  // core method which does not lock on mutex
};

class SerialException : public std::runtime_error {
 public:
  SerialException() : SerialException("generic fault") {}
  explicit SerialException(const std::string &what_arg) : std::runtime_error("Serial Exception: " + what_arg + ".") {}
};

class SerialInvalidArgumentException : public std::invalid_argument {
 public:
  SerialInvalidArgumentException() : SerialInvalidArgumentException("generic fault") {}
  explicit SerialInvalidArgumentException(const std::string &what_arg) : std::invalid_argument("Serial Invalid Argument Exception: " + what_arg + ".") {}
};

class SerialIOException : public std::runtime_error {
 public:
  SerialIOException() : SerialIOException("generic fault") {}
  explicit SerialIOException(const std::string &what_arg) : std::runtime_error("Serial IO Exception: " + what_arg + ", errno has been set to '" + std::to_string(errno) + "'.") {}
};

class SerialPortNotOpenException : public std::runtime_error {
 public:
  SerialPortNotOpenException() : std::runtime_error("Serial Port Not Open Exception.") {}
};

/*!
 * Structure that describes a serial device.
 */
class PortInfo {
 public:
  PortInfo() = default;
  ~PortInfo() = default;

  /*! Address of the serial port (this can be passed to the constructor of Serial). */
  std::string serial_port;

  uint16_t busnum {0};
  uint16_t devnum {0};
  uint16_t id_product {0};
  uint16_t id_vendor {0};
  std::string manufacturer;
  std::string product;
  std::string serial_number;

#if defined(__linux__)
  int getPortInfo(const std::string &serial_port_name);
#endif
};

#if defined(__linux__)
int getLinkPath(std::string system_path, std::string &link_path);
#endif
int getPortsInfo(std::vector<PortInfo> &serial_ports);
int getPortsList(std::vector<std::string> &serial_port_names);
} // namespace serial

#endif
