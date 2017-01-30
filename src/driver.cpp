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

#include "lidar_lite/driver.h"

#include <simple_io/i2c.h>

#include <ros/ros.h>

namespace lidar_lite {

enum ControlRegisters : uint8_t {
  ACQ_COMMAND              = 0x00, // W    [--    ] Device command
  STATUS                   = 0x01, // R    [--    ] System status
  SIG_COUNT_VAL            = 0x02, // R/W  [0x80  ] Maximum acquisition count
  SIG_COUNT_VAL_DEFAULT    = 0x80,
  ACQ_CONFIG_REG           = 0x04, // R/W  [0x08  ] Acquisition mode control
  ACQ_CONFIG_REG_DEFAULT   = 0x08,
  VELOCITY                 = 0x09, // R    [--    ] Velocity measurement output
  PEAK_CORR                = 0x0c, // R    [--    ] Peak value in correlation record
  NOISE_PEAK               = 0x0d, // R    [--    ] Correlation record noise floor
  SIGNAL_STRENGTH          = 0x0e, // R    [--    ] Received signal strength
  FULL_DELAY_HIGH          = 0x0f, // R    [--    ] Distance measurement high byte
  FULL_DELAY_LOW           = 0x10, // R    [--    ] Distance measurement low byte
  OUTER_LOOP_COUNT         = 0x11, // R/W  [0x01  ] Burst measurement count control
  REF_COUNT_VAL            = 0x12, // R/W  [0x05  ] Reference acquisition count
  LAST_DELAY_HIGH          = 0x14, // R    [--    ] Previous distance measurement high byte
  LAST_DELAY_LOW           = 0x15, // R    [--    ] Previous distance measurement low byte
  UNIT_ID_HIGH             = 0x16, // R    [Unique] Serial number high byte
  UNIT_ID_LOW              = 0x17, // R    [Unique] Serial number low byte
  I2C_ID_HIGH              = 0x18, // W    [--    ] Write serial number high byte for I2C address unlock
  I2C_ID_LOW               = 0x19, // W    [--    ] Write serial number low byte for I2C address unlock
  I2C_SEC_ADDR             = 0x1a, // R/W  [--    ] Write new I2C address after unlock
  THRESHOLD_BYPASS         = 0x1c, // R/W  [0x00  ] Peak detection threshold bypass
  THRESHOLD_BYPASS_DEFAULT = 0x00,
  I2C_CONFIG               = 0x1e, // R/W  [0x00  ] Default address response control
  COMMAND                  = 0x40, // R/W  [--    ] State command
  MEASURE_DELAY            = 0x45, // R/W  [0x14  ] Delay between automatic measurements
  PEAK_BCK                 = 0x4c, // R    [--    ] Second largest peak value in correlation record
  CORR_DATA                = 0x52, // R    [--    ] Correlation record data low byte
  CORR_DATA_SIGN           = 0x53, // R    [--    ] Correlation record data high byte
  ACQ_SETTINGS             = 0x5d, // R/W  [--    ] Correlation record memory bank select
  POWER_CONTROL            = 0x65, // R/W  [0x80  ] Power state control
};

enum ReadFlags : uint8_t {
  // Setting the most significant bit of the I2C address byte to one triggers
  // automatic incrementing of the register address with successive reads or writes
  // within an I2C block transfer. This is commonly used to read the two bytes of a
  // 16-bit value within one transfer.
  READ_AUTO_INC_ADDR = 1 << 7,
};

enum class AcqCommand : uint8_t {
  RESET                        = 0x00, // Reset FPGA, all registers return to default values
  MEASURE_WO_BIAS_CORRECTION   = 0x03, // Take distance measurement without receiver bias correction
  MEASURE_WITH_BIAS_CORRECTION = 0x04, // Take distance measurement with receiver bias correction
};

enum class StatusFlags : uint8_t {
  PROCESS_ERROR = 1 << 6,
  // 0: No error detected
  // 1: System error detected during measurement

  HEALTH = 1 << 5,
  // 0: Error detected
  // 1: Reference and receiver bias are operational

  SECONDARY_RETURN = 1 << 4,
  // 0: No secondary return detected
  // 1: Secondary return detected in correlation record

  INVALID_SIGNAL = 1 << 3,
  // 0: Peak detected
  // 1: Peak not detected in correlation record, measurement is invalid

  SIGNAL_OVERFLOW = 1 << 2,
  // 0: Signal data has not overflowed
  // 1: Signal data in correlation record has reached the maximum value before overflow.
  //    This occurs with a strong received signal strength

  REFERENCE_OVERFLOW = 1 << 1,
  // 0: Reference data has not overflowed
  // 1: Reference data in correlation record has reached the maximum value before overflow.
  //    This occurs periodically

  BUSY = 1 << 0,
  // 0: Device is ready for new command
  // 1: Device is busy taking a measurement
};

uint8_t operator &(uint8_t value, StatusFlags flags) {
  return value & (uint8_t)flags;
}

uint8_t operator &(StatusFlags value, StatusFlags flags) {
  return (uint8_t)value & (uint8_t)flags;
}

std::string to_string(StatusFlags flags)
{
  std::string str;
  str.append("{");
  if (flags & StatusFlags::PROCESS_ERROR     ) str.append(" PROCESS_ERROR");
  if (flags & StatusFlags::HEALTH            ) str.append(" HEALTH");
  if (flags & StatusFlags::SECONDARY_RETURN  ) str.append(" SECONDARY_RETURN");
  if (flags & StatusFlags::INVALID_SIGNAL    ) str.append(" INVALID_SIGNAL");
  if (flags & StatusFlags::SIGNAL_OVERFLOW   ) str.append(" SIGNAL_OVERFLOW");
  if (flags & StatusFlags::REFERENCE_OVERFLOW) str.append(" REFERENCE_OVERFLOW");
  if (flags & StatusFlags::BUSY              ) str.append(" BUSY");
  str.append(" }");
  return str;
}

char const* to_cstr(LidarLiteDriver::OperationMode op_mode)
{
#define CASE(NAME) case LidarLiteDriver::OperationMode::NAME: return #NAME;
  switch (op_mode) {
    CASE(DEFAULT);
    CASE(SHORT_RANGE_HIGH_SPEED);
    CASE(DEFAULT_RANGE_HIGHER_SPEED_SHORT_RANGE);
    CASE(MAXIMUM_RANGE);
    CASE(HIGH_SENSITIVITY_DETECTION);
    CASE(LOW_SENSITIVITY_DETECTION);
    default: return "<INVALID>";
  }
#undef CASE
}

/*
 * LidarLiteDriver
 */
LidarLiteDriver::LidarLiteDriver(uint8_t i2c_bus, uint8_t i2c_address)
  : i2c_(new simple_io::I2C(i2c_bus, i2c_address))
{
  ROS_INFO("Created");

  if (!i2c_->init()) {
    ROS_ERROR("Failed to init i2c");
  }

  uint8_t status = 0;
  if (read(STATUS, &status, false)) {
    ROS_INFO("Status: 0x%02x %s", status, to_string((StatusFlags)status).c_str());
    ROS_INFO("UnitID: 0x%04x", uint_id());
  }
  else {
    ROS_ERROR("Failed to read STATUS");
  }
}

LidarLiteDriver::~LidarLiteDriver()
{}

bool
LidarLiteDriver::configure(OperationMode op_mode)
{
  ROS_INFO("Configure with op_mode=%i-%s", (int)op_mode, to_cstr(op_mode));

  switch (op_mode) {
    case OperationMode::DEFAULT:
      write(SIG_COUNT_VAL, SIG_COUNT_VAL_DEFAULT);
      write(ACQ_CONFIG_REG, ACQ_CONFIG_REG_DEFAULT);
      write(THRESHOLD_BYPASS, THRESHOLD_BYPASS_DEFAULT);
      break;

    case OperationMode::SHORT_RANGE_HIGH_SPEED:
      write(SIG_COUNT_VAL, 0x1d);
      write(ACQ_CONFIG_REG, ACQ_CONFIG_REG_DEFAULT);
      write(THRESHOLD_BYPASS, THRESHOLD_BYPASS_DEFAULT);
      break;

    case OperationMode::DEFAULT_RANGE_HIGHER_SPEED_SHORT_RANGE:
      write(SIG_COUNT_VAL, SIG_COUNT_VAL_DEFAULT);
      write(ACQ_CONFIG_REG, 0x00);
      write(THRESHOLD_BYPASS, THRESHOLD_BYPASS_DEFAULT);
      break;

    case OperationMode::MAXIMUM_RANGE:
      write(SIG_COUNT_VAL, 0xff);
      write(ACQ_CONFIG_REG, ACQ_CONFIG_REG_DEFAULT);
      write(THRESHOLD_BYPASS, THRESHOLD_BYPASS_DEFAULT);
      break;

    case OperationMode::HIGH_SENSITIVITY_DETECTION:
      write(SIG_COUNT_VAL, SIG_COUNT_VAL_DEFAULT);
      write(ACQ_CONFIG_REG, ACQ_CONFIG_REG_DEFAULT);
      write(THRESHOLD_BYPASS, 0x80);
      break;

    case OperationMode::LOW_SENSITIVITY_DETECTION:
      write(SIG_COUNT_VAL, SIG_COUNT_VAL_DEFAULT);
      write(ACQ_CONFIG_REG, ACQ_CONFIG_REG_DEFAULT);
      write(THRESHOLD_BYPASS, 0xb0);
      break;
  }

  return true;
}

bool
LidarLiteDriver::reset()
{
  return write(ACQ_COMMAND, (uint8_t)AcqCommand::RESET);
}

uint16_t
LidarLiteDriver::uint_id()
{
  uint16_t uint_id = -1;
  if (!read(UNIT_ID_HIGH | READ_AUTO_INC_ADDR, &uint_id, false)) {
    ROS_ERROR("Failed to read distance");
    return 0;
  }

  uint_id = (uint_id << 8) | (uint_id >> 8);
  ROS_DEBUG("UNIT_ID: 0x%04x", uint_id);

  return uint_id;
}

boost::optional<LidarLiteDriver::centimeters>
LidarLiteDriver::distance(bool bias_correction)
{
  if (!write(ACQ_COMMAND, bias_correction
             ? (uint8_t)AcqCommand::MEASURE_WITH_BIAS_CORRECTION
             : (uint8_t)AcqCommand::MEASURE_WO_BIAS_CORRECTION
  )) {
    ROS_ERROR("Failed to read distance - setting bias correction");
    return {};
  }

  uint16_t distance = -1;
  if (!read(FULL_DELAY_HIGH | READ_AUTO_INC_ADDR, &distance, true)) {
    ROS_ERROR("Failed to read distance");
    return {};
  }

  distance = (distance << 8) | (distance >> 8);
  ROS_DEBUG("Distance: %u cm", distance);

  return centimeters{distance};
}

bool
LidarLiteDriver::write(uint8_t addr, uint8_t value)
{
  if (!i2c_->write(addr, value)) {
    ROS_ERROR("Failed to write 0x%02x to 0x%02x", value, addr);
    return false;
  }

  return true;
}

bool
LidarLiteDriver::read(uint8_t addr, uint8_t* value, size_t bytes, bool monitor_busy_flag)
{
  bool busy_flag = monitor_busy_flag;
  for (size_t count = 0; count < 100 && busy_flag; ++count) {
    uint8_t status;
    if (!i2c_->read(STATUS, &status)) {
      ROS_ERROR("Failed to read STATUS register");
      return false;
    }
    ROS_DEBUG("Status: 0x%02x %s", status, to_string((StatusFlags)status).c_str());
    busy_flag = !!(status & StatusFlags::BUSY);
    usleep(1000);
  }

  if (busy_flag) {
    ROS_ERROR("Failed to read %zu byte(s) from 0x%02x - busy", bytes, addr);
    return false;
  }

  if (!i2c_->read(addr, value, bytes)) {
    ROS_ERROR("Failed to read %zu byte(s) from 0x%02x", bytes, addr);
    return false;
  }

  ROS_DEBUG("Successfully read %zu byte(s) from 0x%02x", bytes, addr);
  return true;
}

} // namespace lidar_lite
