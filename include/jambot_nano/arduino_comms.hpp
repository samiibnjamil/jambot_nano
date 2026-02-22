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
  void reset_encoders();
  void set_pid_values(int k_p, int k_d, int k_i, int k_o);

private:
  LibSerial::BaudRate convert_baud_rate(int baud_rate);
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

} // namespace jambot_nano

#endif // JAMBOT_NANO_ARDUINO_COMMS_HPP
