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

#include <serial/serial.h>
#include <serial/impl/impl.h>

using namespace serial;

class Serial::ScopedReadLock {
 public:
  ScopedReadLock(SerialImpl *pimpl)
      : pimpl_(pimpl) {
    this->pimpl_->readLock();
  }
  ~ScopedReadLock() {
    this->pimpl_->readUnlock();
  }

 private:
  // Disable copy constructors
  ScopedReadLock(const ScopedReadLock &);
  const ScopedReadLock &operator=(ScopedReadLock);

  SerialImpl *pimpl_;
};

class Serial::ScopedWriteLock {
 public:
  ScopedWriteLock(SerialImpl *pimpl)
      : pimpl_(pimpl) {
    this->pimpl_->writeLock();
  }
  ~ScopedWriteLock() {
    this->pimpl_->writeUnlock();
  }

 private:
  // Disable copy constructors
  ScopedWriteLock(const ScopedWriteLock &);
  const ScopedWriteLock &operator=(ScopedWriteLock);

  SerialImpl *pimpl_;
};

Serial::Serial(const std::string &port, uint32_t baudrate, Serial::Timeout timeout, bytesize_t bytesize, parity_t parity,
               stopbits_t stopbits, flowcontrol_t flowcontrol)
    : pimpl_(new SerialImpl(port, baudrate, bytesize, parity, stopbits, flowcontrol)) {
  pimpl_->setTimeout(timeout);
}

Serial::~Serial() {
  delete pimpl_;
}

void Serial::open() {
  pimpl_->open();
}

void Serial::close() {
  pimpl_->close();
}

bool Serial::isOpen() const {
  return pimpl_->isOpen();
}

size_t Serial::available() {
  return pimpl_->available();
}

bool Serial::waitReadable() {
  Serial::Timeout timeout(pimpl_->getTimeout());
  return pimpl_->waitReadable(timeout.getReadConstant());
}

void Serial::waitByteTimes(size_t count) {
  pimpl_->waitByteTimes(count);
}

size_t Serial::read_(uint8_t *buffer, size_t size) {
  return this->pimpl_->read(buffer, size);
}

size_t Serial::read(uint8_t *buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  return this->pimpl_->read(buffer, size);
}

size_t Serial::read(std::vector<uint8_t> &buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  auto buffer_ = new uint8_t[size];
  size_t bytes_read = 0;

  try {
    bytes_read = this->pimpl_->read(buffer_, size);
  } catch (const std::exception &e) {
    delete[] buffer_;
    throw;
  }

  buffer.insert(buffer.end(), buffer_, buffer_ + bytes_read);
  delete[] buffer_;
  return bytes_read;
}

size_t Serial::read(std::string &buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  auto buffer_ = new uint8_t[size];
  size_t bytes_read = 0;
  try {
    bytes_read = this->pimpl_->read(buffer_, size);
  } catch (const std::exception &e) {
    delete[] buffer_;
    throw;
  }
  buffer.append(reinterpret_cast<const char *>(buffer_), bytes_read);
  delete[] buffer_;
  return bytes_read;
}

std::string Serial::read(size_t size) {
  std::string buffer;
  this->read(buffer, size);
  return buffer;
}

size_t Serial::readline(std::string &buffer, size_t size, std::string eol) {
  ScopedReadLock lock(this->pimpl_);
  size_t eol_len = eol.length();
  auto buffer_ = new uint8_t[size];
  size_t read_so_far = 0;
  while (true) {
    size_t bytes_read = this->read_(buffer_ + read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      break; // Timeout occurred on reading 1 byte
    }
    if (std::string(reinterpret_cast<const char *>(buffer_ + read_so_far - eol_len), eol_len) == eol) {
      break; // EOL found
    }
    if (read_so_far == size) {
      break; // Reached the maximum read length
    }
  }
  buffer.append(reinterpret_cast<const char *> (buffer_), read_so_far);
  delete[] buffer_;
  return read_so_far;
}

std::string Serial::readline(size_t size, std::string eol) {
  std::string buffer;
  this->readline(buffer, size, eol);
  return buffer;
}

std::vector<std::string> Serial::readlines(size_t size, std::string eol) {
  ScopedReadLock lock(this->pimpl_);
  std::vector<std::string> lines;
  size_t eol_len = eol.length();
  auto buffer_ = new uint8_t[size];
  size_t read_so_far = 0;
  size_t start_of_line = 0;
  while (read_so_far < size) {
    size_t bytes_read = this->read_(buffer_ + read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      if (start_of_line != read_so_far) {
        lines.push_back(std::string(reinterpret_cast<const char *>(buffer_ + start_of_line), read_so_far - start_of_line));
      }
      break; // Timeout occured on reading 1 byte
    }
    if (std::string(reinterpret_cast<const char *>(buffer_ + read_so_far - eol_len), eol_len) == eol) {
      // EOL found
      lines.push_back(std::string(reinterpret_cast<const char *>(buffer_ + start_of_line), read_so_far - start_of_line));
      start_of_line = read_so_far;
    }
    if (read_so_far == size) {
      if (start_of_line != read_so_far) {
        lines.push_back(std::string(reinterpret_cast<const char *>(buffer_ + start_of_line), read_so_far - start_of_line));
      }
      break; // Reached the maximum read length
    }
  }
  delete[] buffer_;
  return lines;
}

size_t Serial::write(const std::string &data) {
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(reinterpret_cast<const uint8_t *>(data.c_str()), data.length());
}

size_t Serial::write(const std::vector<uint8_t> &data) {
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(&data[0], data.size());
}

size_t Serial::write(const uint8_t *data, size_t size) {
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(data, size);
}

size_t Serial::write_(const uint8_t *data, size_t length) {
  return pimpl_->write(data, length);
}

void Serial::setPort(const std::string &port) {
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  bool was_open = pimpl_->isOpen();
  if (was_open) {
    close();
  }
  pimpl_->setPort(port);
  if (was_open) {
    open();
  }
}

std::string Serial::getPort() const {
  return pimpl_->getPort();
}

void Serial::setTimeout(Serial::Timeout &timeout) {
  pimpl_->setTimeout(timeout);
}

Serial::Timeout Serial::getTimeout() const {
  return pimpl_->getTimeout();
}

void Serial::setBaudrate(uint32_t baudrate) {
  pimpl_->setBaudrate(baudrate);
}

uint32_t Serial::getBaudrate() const {
  return uint32_t(pimpl_->getBaudrate());
}

void Serial::setBytesize(bytesize_t bytesize) {
  pimpl_->setBytesize(bytesize);
}

bytesize_t Serial::getBytesize() const {
  return pimpl_->getBytesize();
}

void Serial::setParity(parity_t parity) {
  pimpl_->setParity(parity);
}

parity_t Serial::getParity() const {
  return pimpl_->getParity();
}

void Serial::setStopbits(stopbits_t stopbits) {
  pimpl_->setStopbits(stopbits);
}

stopbits_t Serial::getStopbits() const {
  return pimpl_->getStopbits();
}

void Serial::setFlowcontrol(flowcontrol_t flowcontrol) {
  pimpl_->setFlowcontrol(flowcontrol);
}

flowcontrol_t Serial::getFlowcontrol() const {
  return pimpl_->getFlowcontrol();
}

void Serial::flush() {
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  pimpl_->flush();
}

void Serial::flushInput() {
  ScopedReadLock lock(this->pimpl_);
  pimpl_->flushInput();
}

void Serial::flushOutput() {
  ScopedWriteLock lock(this->pimpl_);
  pimpl_->flushOutput();
}

void Serial::sendBreak(int duration) {
  pimpl_->sendBreak(duration);
}

void Serial::setBreak(bool level) {
  pimpl_->setBreak(level);
}

void Serial::setRTS(bool level) {
  pimpl_->setRTS(level);
}

void Serial::setDTR(bool level) {
  pimpl_->setDTR(level);
}

bool Serial::waitForChange() {
  return pimpl_->waitForChange();
}

bool Serial::getCTS() {
  return pimpl_->getCTS();
}

bool Serial::getDSR() {
  return pimpl_->getDSR();
}

bool Serial::getRI() {
  return pimpl_->getRI();
}

bool Serial::getCD() {
  return pimpl_->getCD();
}
