#pragma once
#include "cmd_sender_base.h"
#include "cmd_sender_mobile.h"
#include "cmd_sender_rm65b.h"

namespace robot_joy_cmd_factory {
class CmdSenderFactory {
public:
  static std::shared_ptr<robot_joy_cmd_sender::CmdSenderBase> CreateCmdSender(
      const std::string &type) {
    if (type == "mobile") {
      return std::make_shared<robot_joy_cmd_sender::CmdSenderMobile>();
    } else if (type == "rm65b") {
      return std::make_shared<robot_joy_cmd_sender::CmdSenderRM65B>();
    } else {
      return nullptr;
    }
  }
};
} // namespace robot_joy_cmd_factory