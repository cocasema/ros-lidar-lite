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

#include <boost/optional.hpp>

#include <cstdint>
#include <memory>
#include <ratio>
#include <string>

namespace mraa {
class I2c;
}

namespace lidar_lite {

template <typename T, class Ratio = std::ratio<1>>
struct distance {
  T value;
};

/*
 * LidarLiteDriver
 */
class LidarLiteDriver
{
public:
  enum : uint8_t { DEFAULT_I2C_BUS = 1 };
  enum : uint8_t { DEFAULT_I2C_ADDR = 0x62 };

  enum class OperationMode {
    // Default mode, balanced performance.
    DEFAULT = 0,
    // Short range, high speed. Uses 0x1d maximum acquisition count.
    SHORT_RANGE_HIGH_SPEED = 1,
    // Default range, higher speed short range. Turns on quick termination
    // detection for faster measurements at short range (with decreased accuracy)
    DEFAULT_RANGE_HIGHER_SPEED_SHORT_RANGE = 2,
    // Maximum range. Uses 0xff maximum acquisition count.
    MAXIMUM_RANGE = 3,
    // High sensitivity detection. Overrides default valid measurement detection
    // algorithm, and uses a threshold value for high sensitivity and noise.
    HIGH_SENSITIVITY_DETECTION = 4,
    // Low sensitivity detection. Overrides default valid measurement detection
    // algorithm, and uses a threshold value for low sensitivity and noise.
    LOW_SENSITIVITY_DETECTION = 5,
  };

public:
  LidarLiteDriver(uint8_t i2c_bus = DEFAULT_I2C_BUS,
                  uint8_t i2c_address = DEFAULT_I2C_ADDR);
  ~LidarLiteDriver();

  LidarLiteDriver(LidarLiteDriver const&) = delete;
  void operator=(LidarLiteDriver const&) = delete;

  // Selects one of several preset configurations.
  bool configure(OperationMode);

  // Reset device. The device reloads default register settings, including the
  // default I2C address. Re-initialization takes approximately 22ms.
  bool reset();

  using centimeters = lidar_lite::distance<uint16_t, std::centi>;
  boost::optional<centimeters> distance(bool bias_correction);

private:
  bool write(uint8_t addr, uint8_t value);

  bool read(uint8_t addr, size_t bytes, uint8_t* value, bool monitor_busy_flag);

  template <typename Data>
  bool read(uint8_t addr, Data* value, bool monitor_busy_flag) {
    return read(addr, sizeof(*value), static_cast<uint8_t*>(value), monitor_busy_flag);
  }

  uint8_t i2c_bus_;
  uint8_t i2c_address_;
  std::string name_;
  std::unique_ptr<mraa::I2c> i2c_;
};

} // namespace lidar_lite
