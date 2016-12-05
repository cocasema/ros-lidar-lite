/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2016 cocasema
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include <ros/ros.h>

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

namespace io {

class I2C
{
public:
  I2C(uint8_t bus, uint8_t address)
    : bus_(bus)
    , address_(address)
    , file_(0)
  {}

  ~I2C()
  {
    close(file_);
  }

  bool init()
  {
    char filename[20];
    snprintf(filename, sizeof(filename), "/dev/i2c-%u", bus_);

    if ((file_ = open(filename, O_RDWR)) < 0) {
      ROS_ERROR("I2C: Failed to open bus '%s': %i-%s",
                filename, errno, strerror(errno));
      return false;
    }

    if (ioctl(file_, I2C_SLAVE, address_) < 0) {
      ROS_ERROR("I2C: Failed to acquire bus access and/or talk to slave: %i-%s",
                errno, strerror(errno));
      return false;
    }

    ROS_INFO("I2C: Successfully initialized [%s 0x%02x]", filename, address_);
    return true;
  }

  bool write_byte(uint8_t reg, uint8_t data)
  {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    if (write(file_, buf, sizeof(buf)) != sizeof(buf)) {
      ROS_ERROR("I2C: Failed to write byte to register 0x%02x: %i-%s",
                reg, errno, strerror(errno));
      return false;
    }

    return true;
  }

  bool read_bytes(uint8_t reg, uint8_t* data, int len)
  {
    if (write(file_, &reg, sizeof(reg)) != sizeof(reg)) {
      ROS_ERROR("I2C: Failed to read bytes - writing reg=0x%02x: %i-%s",
                reg, errno, strerror(errno));
      return false;
    }

    if (read(file_, data, len) != len) {
      ROS_ERROR("I2C: Failed to read bytes - reading reg=0x%02x: %i-%s",
                reg, errno, strerror(errno));
      return false;
    }

    return true;
  }

private:
  const uint8_t bus_;
  const uint8_t address_;

  int file_;
};

} // namespace io
