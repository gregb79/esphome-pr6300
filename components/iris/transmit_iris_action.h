#pragma once
#include "esphome/core/automation.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include "iris_protocol.h"

namespace esphome {
namespace remote_transmitter {

static const char *const TAG = "transmit_iris";

// map instruction strings to code
int map_instruction(const std::string &i) {
  if (i == "Power") return 17;
  if (i == "Blue") return 33;
  if (i == "Magenta") return 34;
  if (i == "Red") return 35;
  if (i == "Lime") return 36;
  if (i == "Green") return 37;
  if (i == "Aqua") return 38;
  if (i == "White") return 39;
  if (i == "Mode1") return 49;
  if (i == "Mode2") return 50;
  if (i == "Mode3") return 51;
  if (i == "Mode4") return 52;
  if (i == "Brightness") return 65;
  return 17;  // default Power
}

// map mode strings to code
int map_mode(const std::string &m) {
  if (m == "pool") return 1;
  if (m == "spa") return 2;
  if (m == "poolspa") return 3;
  return 3;  // default poolspa
}

class TransmitIrisAction : public Action<> {
 public:
  void set_id0(int id0) { id0_ = id0; }
  void set_id1(int id1) { id1_ = id1; }
  void set_mode(const std::string &mode) { mode_ = map_mode(mode); }
  void set_instruction(const std::string &instr) { instr_ = map_instruction(instr); }
  void set_parent(IrisProtocol *proto) { proto_ = proto; }

  void play() override {
    if (proto_ == nullptr) return;
    ESP_LOGI(TAG, "Sending Iris packet: ID0=%d ID1=%d Mode=%d Instr=%d",
             id0_, id1_, mode_, instr_);
    proto_->send_frame(id0_, id1_, instr_, mode_);
  }

 protected:
  int id0_{249};      // default
  int id1_{203};      // default
  int mode_{3};       // default poolspa
  int instr_{17};     // default Power

  IrisProtocol *proto_{nullptr};
};

}  // namespace remote_transmitter
}  // namespace esphome
