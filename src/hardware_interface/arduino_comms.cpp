#include "jambot_nano/arduino_comms.hpp"
#include <sstream>
#include <limits>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>

namespace jambot_nano
{

ArduinoComms::ArduinoComms()
: timeout_ms_(1000)
{
}

LibSerial::BaudRate ArduinoComms::convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

void ArduinoComms::connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  timeout_ms_ = timeout_ms;
  try {
    serial_conn_.Open(serial_device);
  } catch (const LibSerial::OpenFailed &) {
      std::cerr << "Failed to open Arduino at known path. Trying USB ports..." << std::endl;
      
      // Fall back to trying USB ports
      for (int i = 0; i <= 10; ++i) {
        std::string usb_port = "/dev/ttyUSB" + std::to_string(i);
        try {
          serial_conn_.Open(usb_port);
          std::cout << "Connected to " << usb_port << std::endl;
          break;
        } catch (const LibSerial::OpenFailed &) {
          std::cerr << "Failed to open " << usb_port << ". Trying next port..." << std::endl;
        }
      }
  }
  serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
}

void ArduinoComms::disconnect()
{
  serial_conn_.Close();
}

bool ArduinoComms::connected() const
{
  return serial_conn_.IsOpen();
}

std::string ArduinoComms::send_msg(const std::string &msg_to_send, bool print_output)
{
  serial_conn_.FlushIOBuffers(); // Just in case
  serial_conn_.Write(msg_to_send);

  std::string response = "";
  try {
    // Responses end with \r\n so we will read up to (and including) the \n.
    serial_conn_.ReadLine(response, '\n', timeout_ms_);
  } catch (const LibSerial::ReadTimeout&) {
    std::cerr << "The ReadByte() call has timed out " << std::endl;
  }

  if (print_output) {
    std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
  }

  return response;
}

void ArduinoComms::send_empty_msg()
{
  std::string response = send_msg("\n\r");
}

void ArduinoComms::read_encoder_values(int &val_1, int &val_2)
{
  std::string response = send_msg("e\n\r");
  std::string delimiter = " ";
  size_t del_pos = response.find(delimiter);
  std::string token_1 = response.substr(0, del_pos);
  std::string token_2 = response.substr(del_pos + delimiter.length());

  val_1 = std::atoi(token_1.c_str());
  val_2 = std::atoi(token_2.c_str());
}

void ArduinoComms::set_motor_values(int val_1, int val_2)
{
  std::stringstream ss;
  ss << "m " << val_1 << " " << val_2 << "\n\r";
  send_msg(ss.str());
}

void ArduinoComms::set_led_state(const std::string &color, bool blink)
{
  std::stringstream ss;
  ss << "l " << color << (blink ? " B" : "") << "\n\r";
  send_msg(ss.str());
}

void ArduinoComms::play_sound(int sound_type)
{
  std::stringstream ss;
  ss << "n " << sound_type << "\n\r";
  send_msg(ss.str());
}

void ArduinoComms::read_imu_data(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
  std::string response = send_msg("i\n\r");
  std::istringstream iss(response);
  iss >> ax >> ay >> az >> gx >> gy >> gz;
}

float ArduinoComms::read_battery_voltage()
{
  std::string response = send_msg("b\n\r");
  try
  {
    // Accept both plain numeric responses ("10.96") and prefixed responses ("b 10.96").
    const size_t number_start = response.find_first_of("+-0123456789.");
    if (number_start == std::string::npos) {
      throw std::invalid_argument("no numeric value found");
    }
    return std::stof(response.substr(number_start));
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ArduinoComms"),
      "Invalid battery response '%s': %s",
      response.c_str(),
      ex.what());
    return std::numeric_limits<float>::quiet_NaN();
  }
}

void ArduinoComms::reset_encoders()
{
  send_msg("r\n\r");
}

void ArduinoComms::set_pid_values(int k_p, int k_d, int k_i, int k_o)
{
  std::stringstream ss;
  ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\n\r";
  send_msg(ss.str());
}

} // namespace jambot_nano 
