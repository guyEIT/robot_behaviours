#include "meca500_hardware/meca500_hardware.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

using meca500_hardware::SuccessCode;

namespace meca500_hardware
{

  hardware_interface::CallbackReturn
  Meca500Hardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    robot_ip_ = info.hardware_parameters.at("robot_ip");
    robot_port_ = std::stoi(info.hardware_parameters.at("robot_port"));
    if (const auto connect_timeout = info.hardware_parameters.find("connect_timeout_ms");
        connect_timeout != info.hardware_parameters.end())
    {
      connect_timeout_ms_ = std::stoi(connect_timeout->second);
    }
    if (const auto response_timeout = info.hardware_parameters.find("response_timeout_ms");
        response_timeout != info.hardware_parameters.end())
    {
      response_timeout_ms_ = std::stoi(response_timeout->second);
    }
    if (const auto auto_home = info.hardware_parameters.find("auto_home");
        auto_home != info.hardware_parameters.end())
    {
      // Accept "true" / "1" (case-insensitive) — anything else, including
      // an empty string, leaves auto_home_ at its default (false).
      std::string v = auto_home->second;
      std::transform(v.begin(), v.end(), v.begin(),
                     [](unsigned char c) { return std::tolower(c); });
      auto_home_ = (v == "true" || v == "1" || v == "yes" || v == "on");
    }

    size_t num_joints = info.joints.size();
    hw_states_pos_.resize(num_joints, 0.0);
    hw_commands_pos_.resize(num_joints, 0.0);

    has_gripper_ = (num_joints > NUM_ARM_JOINTS);

    RCLCPP_INFO(
        rclcpp::get_logger("Meca500Hardware"),
        "Configured robot connection to %s:%d (connect timeout: %d ms, response timeout: %d ms, gripper: %s, auto_home: %s)",
        robot_ip_.c_str(),
        robot_port_,
        connect_timeout_ms_,
        response_timeout_ms_,
        has_gripper_ ? "yes" : "no",
        auto_home_ ? "yes" : "no");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  Meca500Hardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < hw_states_pos_.size(); i++)
    {
      state_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
    }
    // Gripper also needs a velocity state interface for GripperActionController
    if (has_gripper_ && NUM_ARM_JOINTS < info_.joints.size())
    {
      state_interfaces.emplace_back(
          info_.joints[NUM_ARM_JOINTS].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  Meca500Hardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < hw_commands_pos_.size(); i++)
    {
      command_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn
  Meca500Hardware::on_activate(const rclcpp_lifecycle::State &)
  {
    if (!connect())
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Meca500Hardware"),
          "Failed to connect to robot at %s:%d",
          robot_ip_.c_str(),
          robot_port_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Meca500 startup sequence
    if (!activateRobot())
    {
      RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"), "Failed to activate robot");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (auto_home_)
    {
      if (!homeRobot())
      {
        RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"), "Failed to home robot");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(
          rclcpp::get_logger("Meca500Hardware"),
          "auto_home=false: skipping Home command. The Meca500 retains its "
          "homing state across power cycles, so once-per-power-on homing is "
          "sufficient. Set auto_home:=true on launch to re-enable.");
    }

    if (has_gripper_)
    {
      RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"), "Initializing gripper...");
      sendCommand("SetGripperForce(40)");
      receiveResponse();
      sendCommand("SetGripperVel(50)");
      receiveResponse();
      sendCommand("GripperOpen");
      receiveResponse();
      // Set initial gripper state to fully open
            hw_states_pos_[NUM_ARM_JOINTS] = GRIPPER_MAX_STROKE_M;
      hw_commands_pos_[NUM_ARM_JOINTS] = GRIPPER_MAX_STROKE_M;
      last_gripper_command_ = GRIPPER_MAX_STROKE_M;
      RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"), "Gripper initialized (open)");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  Meca500Hardware::on_deactivate(const rclcpp_lifecycle::State &)
  {
    deactivateRobot();
    disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

hardware_interface::return_type
Meca500Hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(socket_mutex_);

    sendCommand("GetJoints");
    std::string response = receiveResponse();

    // Look for the first '[' after the timestamp
    auto start = response.find('[');
    if (start != std::string::npos)
    {
        start = response.find('[', start + 1);
        auto end = response.find(']', start);
        if (end != std::string::npos)
        {
            std::string joints_str = response.substr(start + 1, end - start - 1);
            std::istringstream ss(joints_str);
            std::string token;
            size_t i = 0;

            while (std::getline(ss, token, ',') && i < hw_states_pos_.size())
            {
                // Remove whitespace
                token.erase(remove_if(token.begin(), token.end(), ::isspace), token.end());

                // Only parse numeric tokens
                if (!token.empty() && (isdigit(token[0]) || token[0] == '-' || token[0] == '+'))
                {
                    try
                    {
                        hw_states_pos_[i] = std::stod(token) * M_PI / 180.0;  // convert to radians
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"),
                            "Failed to parse joint %zu: '%s'", i, token.c_str());
                    }
                }
                else
                {
                    RCLCPP_DEBUG(rclcpp::get_logger("Meca500Hardware"),
                        "Ignoring non-numeric token: '%s'", token.c_str());
                }

                i++;
            }
        }
    }

    return hardware_interface::return_type::OK;
}

  hardware_interface::return_type
  Meca500Hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    // Send arm joint command (first 6 joints only)
    std::ostringstream cmd;
    cmd << "MoveJoints(";
    for (size_t i = 0; i < NUM_ARM_JOINTS && i < hw_commands_pos_.size(); i++)
    {
      double degrees = hw_commands_pos_[i] * 180.0 / M_PI;  // convert radians -> degrees
      cmd << degrees;
      if (i < NUM_ARM_JOINTS - 1)
        cmd << ",";
    }
    cmd << ")";
    sendCommand(cmd.str());

    // Send gripper command if changed
    if (has_gripper_ && NUM_ARM_JOINTS < hw_commands_pos_.size())
    {
      double gripper_cmd = hw_commands_pos_[NUM_ARM_JOINTS];
      if (std::abs(gripper_cmd - last_gripper_command_) > 1e-6)
      {
        double mm = gripperMetresToMm(gripper_cmd);
        std::ostringstream grip_cmd;
        grip_cmd << "MoveGripper(" << mm << ")";
        sendCommand(grip_cmd.str());
        last_gripper_command_ = gripper_cmd;

        // Update state to commanded position (open-loop feedback)
        hw_states_pos_[NUM_ARM_JOINTS] = gripper_cmd;
      }
    }

    return hardware_interface::return_type::OK;
  }

  bool Meca500Hardware::activateRobot()
  {
    RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"), "Activating Robot...");
    sendCommand("ActivateRobot");
    return waitForReturnCode(SuccessCode::MOTORS_ACTIVATED, SuccessCode::MOTORS_ALREADY_ACTIVATED); // Wait for activation done
  }
  
  bool Meca500Hardware::homeRobot()
  {
    RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"), "Homing Robot...");
    sendCommand("Home");
    return waitForReturnCode(SuccessCode::HOMING_DONE, SuccessCode::ALREADY_HOMED); // Wait for activation done
  }
  
  bool Meca500Hardware::deactivateRobot()
  {
    RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"), "Deactivating Robot...");
    sendCommand("DeactivateRobot");
    return waitForReturnCode(SuccessCode::MOTORS_DEACTIVATED); // Wait for deactivation done
  }

  /* ---------------- TCP helpers ---------------- */

  bool Meca500Hardware::connect()
  {
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Meca500Hardware"),
          "Failed to create robot socket: %s",
          std::strerror(errno));
      return false;
    }

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(robot_port_);
    if (inet_pton(AF_INET, robot_ip_.c_str(), &server.sin_addr) != 1)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Meca500Hardware"),
          "Invalid robot IP address '%s'",
          robot_ip_.c_str());
      disconnect();
      return false;
    }

    const int original_flags = fcntl(socket_fd_, F_GETFL, 0);
    if (original_flags < 0 || fcntl(socket_fd_, F_SETFL, original_flags | O_NONBLOCK) < 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Meca500Hardware"),
          "Failed to configure non-blocking connect for robot socket: %s",
          std::strerror(errno));
      disconnect();
      return false;
    }

    const int connect_result = ::connect(socket_fd_, reinterpret_cast<sockaddr *>(&server), sizeof(server));
    if (connect_result < 0 && errno != EINPROGRESS)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Meca500Hardware"),
          "Robot connect to %s:%d failed immediately: %s",
          robot_ip_.c_str(),
          robot_port_,
          std::strerror(errno));
      disconnect();
      return false;
    }

    if (connect_result < 0)
    {
      fd_set write_fds;
      FD_ZERO(&write_fds);
      FD_SET(socket_fd_, &write_fds);

      timeval timeout{};
      timeout.tv_sec = connect_timeout_ms_ / 1000;
      timeout.tv_usec = (connect_timeout_ms_ % 1000) * 1000;

      const int ready = select(socket_fd_ + 1, nullptr, &write_fds, nullptr, &timeout);
      if (ready <= 0)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("Meca500Hardware"),
            "Timed out connecting to robot at %s:%d after %d ms",
            robot_ip_.c_str(),
            robot_port_,
            connect_timeout_ms_);
        disconnect();
        return false;
      }

      int socket_error = 0;
      socklen_t error_length = sizeof(socket_error);
      if (getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &socket_error, &error_length) < 0 || socket_error != 0)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("Meca500Hardware"),
            "Robot connect to %s:%d failed: %s",
            robot_ip_.c_str(),
            robot_port_,
            std::strerror(socket_error == 0 ? errno : socket_error));
        disconnect();
        return false;
      }
    }

    if (fcntl(socket_fd_, F_SETFL, original_flags) < 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Meca500Hardware"),
          "Failed to restore robot socket flags: %s",
          std::strerror(errno));
      disconnect();
      return false;
    }

    if (!configureSocketTimeouts())
    {
      disconnect();
      return false;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("Meca500Hardware"),
        "Connected to robot at %s:%d",
        robot_ip_.c_str(),
        robot_port_);
    return true;
  }

  void Meca500Hardware::disconnect()
  {
    if (socket_fd_ >= 0)
    {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  bool Meca500Hardware::configureSocketTimeouts()
  {
    timeval timeout{};
    timeout.tv_sec = response_timeout_ms_ / 1000;
    timeout.tv_usec = (response_timeout_ms_ % 1000) * 1000;

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0 ||
        setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("Meca500Hardware"),
          "Failed to configure robot socket timeouts: %s",
          std::strerror(errno));
      return false;
    }

    return true;
  }

  bool Meca500Hardware::sendCommand(const std::string &cmd)
  {
    std::string msg = cmd + "\n";
    if (send(socket_fd_, msg.c_str(), msg.size(), 0) < 0)
    {
      RCLCPP_WARN(
          rclcpp::get_logger("Meca500Hardware"),
          "Failed to send robot command '%s': %s",
          cmd.c_str(),
          std::strerror(errno));
      return false;
    }
    return true;
  }

  std::string Meca500Hardware::receiveResponse()
  {
    char buffer[1024];
    ssize_t len = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
    if (len > 0)
    {
      buffer[len] = '\0';
      return std::string(buffer);
    }
    if (len == 0)
    {
      RCLCPP_WARN(
          rclcpp::get_logger("Meca500Hardware"),
          "Robot socket closed while waiting for response");
    }
    else
    {
      RCLCPP_WARN(
          rclcpp::get_logger("Meca500Hardware"),
          "Timed out or failed while waiting for robot response: %s",
          std::strerror(errno));
    }
    return "";
  }

  bool Meca500Hardware::waitForReturnCode(SuccessCode code1, SuccessCode code2)
  {
    SuccessCode received_code{0};

    while (received_code != code1 && received_code != code2)
    {
      std::string full_code = receiveResponse();
      if (full_code.empty())
      {
        RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"),
                    "No response received while waiting for robot return code");
        return false;
      }
      received_code = parseReturnCode(full_code); 
      
      RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"),
                        "got return code: %s", full_code.c_str());

      // return 1 and break if an error happens
      if (isErrorCode(received_code))
      {
        RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"),
                        "got error code: %s", getErrorMessage(received_code).c_str());
        return false; // failure
      }
    }

    return true; // Success
  }

  SuccessCode Meca500Hardware::parseReturnCode(std::string raw_code)
  {
    if (raw_code.size() < 5)
    {
      RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"),
                  "Malformed robot return string: '%s'", raw_code.c_str());
      return SuccessCode{0};
    }

    try
    {
      return static_cast<meca500_hardware::SuccessCode>(std::stoul(raw_code.substr(1, 4)));
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"),
                  "Failed to parse robot return string '%s': %s",
                  raw_code.c_str(), e.what());
      return SuccessCode{0};
    }
  }

  double Meca500Hardware::gripperMetresToMm(double metres) const
  {
    // Per-finger displacement in metres -> total opening in mm for the Meca API
    return std::clamp(metres / GRIPPER_MAX_STROKE_M * GRIPPER_MAX_STROKE_MM, 0.0, GRIPPER_MAX_STROKE_MM);
  }

  double Meca500Hardware::gripperMmToMetres(double mm) const
  {
    // Total opening in mm from the Meca API -> per-finger displacement in metres
    return std::clamp(mm / GRIPPER_MAX_STROKE_MM * GRIPPER_MAX_STROKE_M, 0.0, GRIPPER_MAX_STROKE_M);
  }

} // namespace meca500_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  meca500_hardware::Meca500Hardware,
  hardware_interface::SystemInterface)
