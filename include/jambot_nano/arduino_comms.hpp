#ifndef JAMBOT_NANO_ARDUINO_COMMS_HPP
#define JAMBOT_NANO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>

namespace jambot_nano
{

class ArduinoComms
{
public:
  ArduinoComms();
  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void disconnect();
  bool connected() const;
  std::string send_msg(const std::string &msg_to_send, bool print_output = false);
  void send_empty_msg();
  void read_encoder_values(int &val_1, int &val_2);
  void set_motor_values(double left_wheel_rad_s, double right_wheel_rad_s);
  void set_led_state(int red, int green, int blue);
  void play_sound(int sound_type);
  void read_imu_data(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  float read_battery_voltage();
  // Combined encoder + IMU + battery read in a single serial round-trip,
  // replacing separate read_encoder_values/read_imu_data/read_battery_voltage
  // calls. imu_ok is false when the firmware's IMU never initialized;
  // callers should skip publishing IMU data for that cycle in that case.
  bool read_telemetry(
    int &enc_1, int &enc_2, bool &imu_ok,
    float &ax, float &ay, float &az, float &gx, float &gy, float &gz,
    float &battery_voltage);
  void reset_encoders();
  // Gains are fractional (e.g. Kp=0.6, Ki=1.7, Kd=0.001) -- these were int,
  // which truncated a real gain set to "0 1 0" and silently destroyed the loop.
  void set_pid_values(double k_p, double k_d, double k_i, double k_o);
  int consecutive_errors() const;
  // Off by default: logs the full firmware telemetry line every read()
  // cycle (20Hz) -- real value while chasing a control-loop bug, real log
  // spam otherwise. Toggle with the "verbose_telemetry" hardware parameter.
  void set_verbose_telemetry(bool enabled) { verbose_telemetry_ = enabled; }

private:
  LibSerial::BaudRate convert_baud_rate(int baud_rate);
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
  int consecutive_errors_ = 0;
  bool verbose_telemetry_ = false;
};

} // namespace jambot_nano

#endif // JAMBOT_NANO_ARDUINO_COMMS_HPP
