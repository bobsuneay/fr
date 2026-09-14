// Copyright 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hkv_gripper_controller/default_serial.hpp"
#include <stdexcept>
#include <cstring>

// Linux/Unix-only includes
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

namespace hkv_gripper_controller
{

struct DefaultSerial::Impl
{
  std::string port;
  uint32_t baudrate;
  double timeout;

  int fd{-1};
  
  Impl(const std::string& p, uint32_t b, double t)
    : port(p), baudrate(b), timeout(t) {}
};

DefaultSerial::DefaultSerial(const std::string& port, uint32_t baudrate, double timeout)
  : impl_(std::make_unique<Impl>(port, baudrate, timeout))
{
}

DefaultSerial::~DefaultSerial()
{
  close();
}

bool DefaultSerial::open()
{
  impl_->fd = ::open(impl_->port.c_str(), O_RDWR | O_NOCTTY);
  
  if (impl_->fd < 0) {
    return false;
  }
  
  struct termios2 tty;
  if (ioctl(impl_->fd, TCGETS2, &tty) != 0) {
    close();
    return false;
  }
  
  // Set arbitrary baud rate
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = impl_->baudrate;
  tty.c_ospeed = impl_->baudrate;
  
  // 8N1
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  
  tty.c_cc[VTIME] = static_cast<cc_t>(impl_->timeout * 10);
  tty.c_cc[VMIN] = 0;
  
  if (ioctl(impl_->fd, TCSETS2, &tty) != 0) {
    close();
    return false;
  }
  
  return true;
}

void DefaultSerial::close()
{
  if (impl_->fd >= 0) {
    ::close(impl_->fd);
    impl_->fd = -1;
  }
}

bool DefaultSerial::isOpen() const
{
  return impl_->fd >= 0;
}

size_t DefaultSerial::write(const std::vector<uint8_t>& data)
{
  if (!isOpen()) {
    return 0;
  }

  ssize_t result = ::write(impl_->fd, data.data(), data.size());
  return result > 0 ? static_cast<size_t>(result) : 0;
}

std::vector<uint8_t> DefaultSerial::read(size_t size)
{
  std::vector<uint8_t> buffer(size);
  
  if (!isOpen()) {
    return {};
  }

  ssize_t result = ::read(impl_->fd, buffer.data(), size);
  if (result > 0) {
    buffer.resize(result);
  } else {
    buffer.clear();
  }
  
  return buffer;
}

size_t DefaultSerial::available() const
{
  if (!isOpen()) {
    return 0;
  }

  int bytes = 0;
  if (ioctl(impl_->fd, FIONREAD, &bytes) == -1) {
    return 0;
  }
  return static_cast<size_t>(bytes);
}

void DefaultSerial::flush()
{
  if (!isOpen()) {
    return;
  }

  ioctl(impl_->fd, TCFLSH, TCIOFLUSH);
}

}  // namespace hkv_gripper_controller
